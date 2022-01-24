// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.oi.OverrideOI.VisionLEDMode;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.CircleFitter;

public class Vision extends SubsystemBase {
  private static final Rotation2d horizontalPlaneToLens =
      Rotation2d.fromDegrees(29.75);
  private static final double lensHeightMeters = Units.inchesToMeters(18.25);
  private static final double circleFitPrecision = 0.01;
  private static final int minTargetCount = 2; // For calculating odometry
  private static final double extraLatencySecs = 0.06; // Approximate camera + network latency
  private static final double targetGraceSecs = 0.5;
  private static final double blinkPeriodSecs = 3.0;
  private static final double blinkLengthSecs = 0.5;

  // FOV constants
  private static final double vpw = 2.0 * Math.tan(Math.toRadians(59.6 / 2.0));
  private static final double vph = 2.0 * Math.tan(Math.toRadians(49.7 / 2.0));

  private final VisionIO io;
  private final VisionIOInputs inputs = new VisionIOInputs();

  private double lastCaptureTimestamp = 0.0;
  private Supplier<VisionLEDMode> modeSupplier;
  private Consumer<TimestampedTranslation2d> translationConsumer;

  private boolean ledsOn = false;
  private boolean forceLeds = false;
  private Timer targetGraceTimer = new Timer();

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
    targetGraceTimer.start();
  }

  public void setOverrides(Supplier<VisionLEDMode> supplier) {
    this.modeSupplier = supplier;
  }

  public void setTranslationConsumer(
      Consumer<TimestampedTranslation2d> consumer) {
    translationConsumer = consumer;
  }

  /** Use to enable LEDs continuously while override is "Auto" */
  public void setForceLeds(boolean on) {
    forceLeds = on;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Vision", inputs);
    int targetCount = ledsOn ? inputs.cornerX.length / 4 : 0;

    // Update LED idle state
    if (targetCount > 0) {
      targetGraceTimer.reset();
    }
    boolean idleOn = targetGraceTimer.get() < targetGraceSecs
        || Timer.getFPGATimestamp() % blinkPeriodSecs < blinkLengthSecs;

    // Update LED state based on switch
    switch (modeSupplier.get()) {
      case ALWAYS_OFF:
        ledsOn = false;
        break;
      case ALWAYS_ON:
        ledsOn = true;
        break;
      case AUTO:
        if (forceLeds) {
          ledsOn = true;
        } else if (DriverStation.isDisabled()) {
          ledsOn = false;
        } else if (DriverStation.isAutonomous()) {
          ledsOn = true;
        } else {
          ledsOn = idleOn;
        }
        break;
      default:
        ledsOn = false;
        break;
    }
    io.setLeds(ledsOn);

    // Exit if no new frame
    if (inputs.captureTimestamp == lastCaptureTimestamp) {
      return;
    }
    lastCaptureTimestamp = inputs.captureTimestamp;

    // Calculate camera to target translation
    if (targetCount >= minTargetCount) {
      List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
      for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
        List<VisionPoint> corners = new ArrayList<>();
        double totalX = 0.0, totalY = 0.0;
        for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
          if (i < inputs.cornerX.length && i < inputs.cornerY.length) {
            corners.add(new VisionPoint(inputs.cornerX[i], inputs.cornerY[i]));
            totalX += inputs.cornerX[i];
            totalY += inputs.cornerY[i];
          }
        }

        VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
        corners = sortCorners(corners, targetAvg);

        for (int i = 0; i < corners.size(); i++) {
          Translation2d translation = solveCameraToTargetTranslation(
              corners.get(i), i < 2 ? FieldConstants.visionTargetHeightUpper
                  : FieldConstants.visionTargetHeightLower);
          if (translation != null) {
            cameraToTargetTranslations.add(translation);
          }
        }
      }

      if (cameraToTargetTranslations.size() >= minTargetCount * 4) {
        Translation2d cameraToTargetTranslation =
            CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
                cameraToTargetTranslations, circleFitPrecision);
        translationConsumer.accept(new TimestampedTranslation2d(
            inputs.captureTimestamp - extraLatencySecs,
            cameraToTargetTranslation));

        List<Double> translationData = new ArrayList<>();
        translationData.add(cameraToTargetTranslation.getX());
        translationData.add(cameraToTargetTranslation.getY());
        for (Translation2d translation : cameraToTargetTranslations) {
          translationData.add(translation.getX());
          translationData.add(translation.getY());
        }
        Logger.getInstance().recordOutput("Vision/TranslationData",
            translationData.stream().mapToDouble(Double::doubleValue)
                .toArray());
      }
    }
  }

  private List<VisionPoint> sortCorners(List<VisionPoint> corners,
      VisionPoint average) {

    // Find top corners
    Integer topLeftIndex = null;
    Integer topRightIndex = null;
    double minPosRads = Math.PI;
    double minNegRads = Math.PI;
    for (int i = 0; i < corners.size(); i++) {
      VisionPoint corner = corners.get(i);
      double angleRad =
          new Rotation2d(corner.x - average.x, average.y - corner.y)
              .minus(Rotation2d.fromDegrees(90)).getRadians();
      if (angleRad > 0) {
        if (angleRad < minPosRads) {
          minPosRads = angleRad;
          topLeftIndex = i;
        }
      } else {
        if (Math.abs(angleRad) < minNegRads) {
          minNegRads = Math.abs(angleRad);
          topRightIndex = i;
        }
      }
    }

    // Find lower corners
    Integer lowerIndex1 = null;
    Integer lowerIndex2 = null;
    for (int i = 0; i < corners.size(); i++) {
      boolean alreadySaved = false;
      if (topLeftIndex != null) {
        if (topLeftIndex.equals(i)) {
          alreadySaved = true;
        }
      }
      if (topRightIndex != null) {
        if (topRightIndex.equals(i)) {
          alreadySaved = true;
        }
      }
      if (!alreadySaved) {
        if (lowerIndex1 == null) {
          lowerIndex1 = i;
        } else {
          lowerIndex2 = i;
        }
      }
    }

    // Combine final list
    List<VisionPoint> newCorners = new ArrayList<>();
    if (topLeftIndex != null) {
      newCorners.add(corners.get(topLeftIndex));
    }
    if (topRightIndex != null) {
      newCorners.add(corners.get(topRightIndex));
    }
    if (lowerIndex1 != null) {
      newCorners.add(corners.get(lowerIndex1));
    }
    if (lowerIndex2 != null) {
      newCorners.add(corners.get(lowerIndex2));
    }
    return newCorners;
  }

  private Translation2d solveCameraToTargetTranslation(VisionPoint corner,
      double goalHeight) {
    double yPixels = corner.x;
    double zPixels = corner.y;

    // Robot frame of reference
    double nY = -((yPixels - 480.0) / 480.0);
    double nZ = -((zPixels - 360.0) / 360.0);

    Translation2d xzPlaneTranslation =
        new Translation2d(1.0, vph / 2.0 * nZ).rotateBy(horizontalPlaneToLens);
    double x = xzPlaneTranslation.getX();
    double y = vpw / 2.0 * nY;
    double z = xzPlaneTranslation.getY();

    double differentialHeight = lensHeightMeters - goalHeight;
    if ((z < 0.0) == (differentialHeight > 0.0)) {
      double scaling = differentialHeight / -z;
      double distance = Math.hypot(x, y) * scaling;
      Rotation2d angle = new Rotation2d(x, y);
      return new Translation2d(distance * angle.getCos(),
          distance * angle.getSin());
    }
    return null;
  }

  public static class VisionPoint {
    public final double x;
    public final double y;

    public VisionPoint(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  public static class TimestampedTranslation2d {
    public final double timestamp;
    public final Translation2d translation;

    public TimestampedTranslation2d(double timestamp,
        Translation2d translation) {
      this.timestamp = timestamp;
      this.translation = translation;
    }
  }
}

