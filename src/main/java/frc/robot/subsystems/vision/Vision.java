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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.oi.OverrideOI.VisionLEDMode;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.CircleFitter;

public class Vision extends SubsystemBase {
  private static final Rotation2d horizontalPlaneToLens =
      Rotation2d.fromDegrees(30.0);
  private static final double lensHeightMeters = Units.inchesToMeters(18.25);
  private static final double circleFitPrecision = 0.01;
  private static final int minTargetCount = 2; // For calculating odometry
  private static final double targetGraceSecs = 0.5;
  private static final double blinkPeriodSecs = 3.0;
  private static final double blinkLengthSecs = 0.2;

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
        List<TargetCorner> corners = new ArrayList<>();
        for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
          corners.add(new TargetCorner(inputs.cornerX[i], inputs.cornerY[i]));
        }

        corners = corners.stream().sorted(sortByY).collect(Collectors.toList());

        for (int i = 0; i < 4; i++) {
          Translation2d translation = solveCameraToTargetTranslation(
              corners.get(0), i < 2 ? FieldConstants.visionTargetHeightUpper
                  : FieldConstants.visionTargetHeightLower);
          if (translation != null) {
            cameraToTargetTranslations.add(translation);
          }
        }
      }

      if (cameraToTargetTranslations.size() > minTargetCount * 4) {
        Translation2d cameraToTargetTranslation =
            CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
                cameraToTargetTranslations, circleFitPrecision);
        translationConsumer.accept(new TimestampedTranslation2d(
            inputs.captureTimestamp, cameraToTargetTranslation));
      }
    }
  }

  private Translation2d solveCameraToTargetTranslation(TargetCorner corner,
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

  private static final Comparator<TargetCorner> sortByY = (TargetCorner c1,
      TargetCorner c2) -> Double.valueOf(c1.y).compareTo(Double.valueOf(c2.y));

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

