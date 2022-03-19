// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.SortedMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.leds.Leds;

/** Manages the robot's pose based on state data from various subsystems. */
public class RobotState {
  private static final double historyLengthSecs = 1.0;
  private static final double maxNoVisionLog = 0.25; // How long to wait with no vision data before
                                                     // clearing log visualization
  private static final double visionShiftPerSec = 0.85; // After one second of vision data, what %
                                                        // of pose average should be vision
  private static final double visionMaxAngularVelocity =
      Units.degreesToRadians(8.0); // Max angular velocity before vision data is rejected
  private static final double ledsAlignedMaxDegrees = 5.0; // How far from fully targeted can the
                                                           // robot be for setting LEDs

  private final TreeMap<Double, Double> hoodData = new TreeMap<>(); // Hood angles
  private final TreeMap<Double, Twist2d> driveData = new TreeMap<>(); // Relative movement per cycle
  private final TreeMap<Double, Translation2d> visionData = new TreeMap<>(); // Field to vehicle

  private Pose2d basePose = new Pose2d();
  private Pose2d latestPose = new Pose2d();
  private boolean resetOnNextVision = false;
  private Leds leds;

  public void setLeds(Leds leds) {
    this.leds = leds;
  }

  /** Records a new hood angle. */
  public void addHoodData(double timestamp, double angle) {
    hoodData.put(timestamp, angle);
    update();
  }

  /** Records a new drive movement. */
  public void addDriveData(double timestamp, Twist2d twist) {
    driveData.put(timestamp, twist);
    update();
  }

  /** Records a new vision frame. */
  public void addVisionData(double timestamp, Translation2d translation) {
    if (resetOnNextVision) {
      resetPose(new Pose2d(translation, getLatestRotation()));
      resetOnNextVision = false;
    } else {
      visionData.put(timestamp, translation);
      update();
    }
  }

  /** Returns the latest robot pose based on drive and vision data. */
  public Pose2d getLatestPose() {
    return latestPose;
  }

  /** Returns the latest robot rotation. */
  public Rotation2d getLatestRotation() {
    return latestPose.getRotation();
  }

  /** Resets the odometry to a known pose. */
  public void resetPose(Pose2d pose) {
    basePose = pose;
    driveData.clear();
    visionData.clear();
    update();
  }

  /** Resets the odometry once the first vision data is received. */
  public void resetOnNextVision() {
    resetOnNextVision = true;
  }

  /** Returns whether the odometry has been reset based on vision data. */
  public boolean getVisionResetComplete() {
    return !resetOnNextVision;
  }

  /** Clears old data and calculates the latest pose. */
  private void update() {
    // Clear old hood data
    while (hoodData.size() > 0
        && hoodData.firstKey() < Timer.getFPGATimestamp() - historyLengthSecs) {
      hoodData.pollFirstEntry();
    }

    // Clear old drive data
    while (driveData.size() > 1 && driveData
        .firstKey() < Timer.getFPGATimestamp() - historyLengthSecs) {
      basePose = getPose(driveData.higherKey(driveData.firstKey()));
      driveData.pollFirstEntry();
    }

    // Clear old vision data
    while (visionData.size() > 0
        && visionData.firstKey() < driveData.firstKey()) {
      // Any vision data before first drive data won't be used in calculations
      visionData.pollFirstEntry();
    }

    // Update latest pose
    latestPose = getPose(null);

    // Log poses
    Logger.getInstance().recordOutput("Odometry/Robot",
        new double[] {latestPose.getX(), latestPose.getY(),
            latestPose.getRotation().getRadians()});
    Map.Entry<Double, Translation2d> visionEntry = visionData.lastEntry();
    if (visionEntry != null) {
      if (visionEntry.getKey() > Timer.getFPGATimestamp() - maxNoVisionLog) {
        Logger.getInstance().recordOutput("Odometry/VisionPose",
            new double[] {visionEntry.getValue().getX(),
                visionEntry.getValue().getY(),
                latestPose.getRotation().getRadians()});
        Logger.getInstance().recordOutput("Odometry/VisionTarget",
            new double[] {FieldConstants.hubCenter.getX(),
                FieldConstants.hubCenter.getY()});
        Logger.getInstance().recordOutput("Vision/DistanceInches",
            Units.metersToInches(
                visionEntry.getValue().getDistance(FieldConstants.hubCenter)));
      }
    }

    // Check if robot is aligned for LEDs
    if (leds != null) {
      leds.setTargeted(
          Math.abs(AutoAim.getTargetRotation(latestPose.getTranslation())
              .minus(latestPose.getRotation())
              .getDegrees()) < ledsAlignedMaxDegrees);
    }
  }

  /**
   * Calculates the pose at the specified timestamp by combining drive and vision data. Data is not
   * interpolated between drive cycles; use getDriveRotation() for precise rotation data.
   */
  public Pose2d getPose(Double timestamp) {

    // Get drive data in range
    SortedMap<Double, Twist2d> filteredDriveData;
    if (timestamp == null) {
      filteredDriveData = driveData;
    } else {
      filteredDriveData = driveData.headMap(timestamp);
    }

    // Process drive and vision data
    Pose2d pose = basePose;
    for (Map.Entry<Double, Twist2d> driveEntry : filteredDriveData.entrySet()) {

      // Get the next timestamp and relevant vision data
      Double nextTimestamp = driveData.higherKey(driveEntry.getKey());
      if (nextTimestamp == null) {
        nextTimestamp = driveEntry.getKey() + Constants.loopPeriodSecs;
      }
      SortedMap<Double, Translation2d> filteredVisionData =
          visionData.subMap(driveEntry.getKey(), nextTimestamp);

      // Apply vision data
      for (Map.Entry<Double, Translation2d> visionEntry : filteredVisionData
          .entrySet()) {

        // Calculate vision shift based on angular velocity
        double angularVelocityRadPerSec = driveEntry.getValue().dtheta
            / (nextTimestamp - driveEntry.getKey());
        double angularErrorScale =
            Math.abs(angularVelocityRadPerSec) / visionMaxAngularVelocity;
        angularErrorScale = MathUtil.clamp(angularErrorScale, 0, 1);
        double visionShift = 1 - Math.pow(1 - visionShiftPerSec,
            1 / VisionConstants.nominalFramerate);
        visionShift *= 1 - angularErrorScale;

        // Adjust pose
        pose = new Pose2d(
            pose.getX() * (1 - visionShift)
                + visionEntry.getValue().getX() * visionShift,
            pose.getY() * (1 - visionShift)
                + visionEntry.getValue().getY() * visionShift,
            pose.getRotation());
      }

      // Apply drive twist
      pose = pose.exp(driveEntry.getValue());
    }
    return pose;
  }

  /** Interpolates to find the hood angle at the specified timestamp. */
  public Optional<Double> getHoodAngle(double timestamp) {
    Map.Entry<Double, Double> floor = hoodData.floorEntry(timestamp);
    Map.Entry<Double, Double> ceiling = hoodData.ceilingEntry(timestamp);

    if (floor == null && ceiling == null) {
      return Optional.empty();
    }
    if (floor == null) {
      return Optional.of(ceiling.getValue());
    }
    if (ceiling == null) {
      return Optional.of(floor.getValue());
    }

    double dydx = (ceiling.getValue() - floor.getValue())
        / (ceiling.getKey() - floor.getKey());
    double y = dydx * (timestamp - floor.getKey()) + floor.getValue();
    return Optional.of(y);
  }

  /**
   * Interpolates to find the drive rotation at the specified timestamp. This only uses the drive
   * data since the gyro angle is unaffected by vision data.
   */
  public Rotation2d getDriveRotation(double timestamp) {
    Rotation2d rotation = basePose.getRotation();
    for (Map.Entry<Double, Twist2d> entry : driveData.headMap(timestamp)
        .entrySet()) {
      Double nextTimestamp = driveData.higherKey(entry.getKey());
      if (nextTimestamp != null && nextTimestamp > timestamp) { // Last twist, apply partial
        double t =
            (timestamp - entry.getKey()) / (nextTimestamp - entry.getKey());
        rotation = rotation.plus(new Rotation2d(entry.getValue().dtheta * t));
      } else { // Apply full twist
        rotation = rotation.plus(new Rotation2d(entry.getValue().dtheta));
      }
    }
    return rotation;
  }
}
