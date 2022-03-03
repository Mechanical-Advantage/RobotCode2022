// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.CameraPosition;
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision.TimestampedTranslation2d;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseHistory;
import frc.robot.util.TunableNumber;

public class Drive extends SubsystemBase {
  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
                                                                   // switch to coast when disabling
  private static final int poseHistoryCapacity = 500;
  private static final double maxNoVisionLog = 0.25; // How long to wait with no vision data before
                                                     // clearing log visualization
  private static final double visionNominalFramerate = 22;
  private static final double visionShiftPerSec = 0.85; // After one second of vision data, what %
  // of pose average should be vision
  private static final double visionMaxAngularVelocity =
      Units.degreesToRadians(8.0); // Max angular velocity before vision data is rejected

  // Thresholding for when to light LEDs and indicate robot is targeted
  private static final double ledsAlignedRadius = 4.0;
  private static final double ledsAlignedMaxDegrees = 5.0;

  private final double wheelRadiusMeters;
  private final double maxVelocityMetersPerSec;
  private final double trackWidthMeters;
  private final SimpleMotorFeedforward leftModel, rightModel;
  private final TunableNumber kP = new TunableNumber("Drive/kP");
  private final TunableNumber kD = new TunableNumber("Drive/kD");

  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();

  private Supplier<Boolean> disableOverride = () -> false;
  private Supplier<Boolean> openLoopOverride = () -> false;
  private Supplier<Boolean> internalEncoderOverride = () -> false;
  private Supplier<HoodState> hoodStateSupplier;
  private Leds leds;

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
  private final Timer noVisionTimer = new Timer(); // Time since last vision update
  private PoseHistory poseHistory = new PoseHistory(poseHistoryCapacity);
  private Pose2d lastVisionPose = new Pose2d();
  private double baseDistanceLeftRad = 0.0;
  private double baseDistanceRightRad = 0.0;
  private boolean brakeMode = false;
  private boolean resetOnVision = false;
  private boolean pitchResetComplete = false;
  private double basePitchRadians = 0.0;

  /** Creates a new DriveTrain. */
  public Drive(DriveIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        wheelRadiusMeters = Units.inchesToMeters(2.0822);
        trackWidthMeters = 0.65778;
        leftModel = new SimpleMotorFeedforward(0.23071, 0.12413);
        rightModel = new SimpleMotorFeedforward(0.23455, 0.12270);
        kP.setDefault(0.00008);
        kD.setDefault(0.0);
        break;
      case ROBOT_2022P:
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        wheelRadiusMeters = Units.inchesToMeters(2.0);
        trackWidthMeters = Units.inchesToMeters(25.528);
        leftModel = new SimpleMotorFeedforward(0.20554, 0.10965, 0.016329);
        rightModel = new SimpleMotorFeedforward(0.20231, 0.11768, 0.0085871);
        kP.setDefault(0.00002);
        kD.setDefault(0.0013);
        break;
      case ROBOT_2020:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        trackWidthMeters = 1.768748;
        leftModel = new SimpleMotorFeedforward(0.23004, 0.2126, 0.036742);
        rightModel = new SimpleMotorFeedforward(0.22652, 0.21748, 0.03177);
        kP.setDefault(0.00015);
        kD.setDefault(0.0015);
        break;
      case ROBOT_KITBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(122.0);
        wheelRadiusMeters = Units.inchesToMeters(3.18);
        trackWidthMeters = 0.6928821;
        leftModel = new SimpleMotorFeedforward(0.75379, 0.25162, 0.042941);
        rightModel = new SimpleMotorFeedforward(0.70773, 0.24745, 0.032956);
        kP.setDefault(2);
        kD.setDefault(40);
        break;
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        trackWidthMeters = 0.7;
        leftModel = new SimpleMotorFeedforward(0.0, 0.22643, 0.018292);
        rightModel = new SimpleMotorFeedforward(0.0, 0.22643, 0.018292);
        kP.setDefault(0.4);
        kD.setDefault(0.0);
        break;
      case ROBOT_ROMI:
        maxVelocityMetersPerSec = 0.6;
        wheelRadiusMeters = 0.035;
        trackWidthMeters = 0.281092;
        leftModel = new SimpleMotorFeedforward(0.27034, 0.64546, 0.021935);
        rightModel = new SimpleMotorFeedforward(0.48548, 0.37427, 0.07421);
        kP.setDefault(0.25);
        kD.setDefault(0.001);
        break;
      default:
        maxVelocityMetersPerSec = 0;
        wheelRadiusMeters = Double.POSITIVE_INFINITY;
        trackWidthMeters = 1.0;
        leftModel = new SimpleMotorFeedforward(0, 0, 0);
        rightModel = new SimpleMotorFeedforward(0, 0, 0);
        kP.setDefault(0);
        kD.setDefault(0);
        break;
    }

    io.setBrakeMode(false);
    noVisionTimer.start();
  }

  /** Set suppliers for external data. */
  public void setSuppliers(Supplier<Boolean> disableOverride,
      Supplier<Boolean> openLoopOverride,
      Supplier<Boolean> internalEncoderOverride,
      Supplier<HoodState> hoodStateSupplier) {
    this.disableOverride = disableOverride;
    this.openLoopOverride = openLoopOverride;
    this.internalEncoderOverride = internalEncoderOverride;
    this.hoodStateSupplier = hoodStateSupplier;
  }

  public void setLeds(Leds leds) {
    this.leds = leds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry
    odometry.update(new Rotation2d(inputs.gyroYawPositionRad * -1),
        getLeftPositionMeters() - baseDistanceLeftRad,
        getRightPositionMeters() - baseDistanceRightRad);

    // Log robot pose
    Pose2d robotPose = odometry.getPoseMeters();
    poseHistory.insert(Timer.getFPGATimestamp(), robotPose);
    Logger.getInstance().recordOutput("Odometry/Robot",
        new double[] {robotPose.getX(), robotPose.getY(),
            robotPose.getRotation().getRadians()});

    // Log vision pose
    if (noVisionTimer.get() < maxNoVisionLog) {
      Logger.getInstance().recordOutput("Odometry/VisionPose",
          new double[] {lastVisionPose.getX(), lastVisionPose.getY(),
              lastVisionPose.getRotation().getRadians()});
      Logger.getInstance().recordOutput("Odometry/VisionTarget", new double[] {
          FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0});
      Logger.getInstance().recordOutput("Vision/DistanceInches",
          Units.metersToInches(lastVisionPose.getTranslation()
              .getDistance(FieldConstants.hubCenter)));
    }

    // Manage pitch rotation
    if (!pitchResetComplete) {
      resetPitch();
      pitchResetComplete = true;
    }
    Logger.getInstance().recordOutput("Drive/PitchDegrees", getPitchDegrees());

    // Check if robot is aligned for LEDs
    boolean withinRadius = getPose().getTranslation()
        .getDistance(FieldConstants.hubCenter) < ledsAlignedRadius;
    boolean withinRotation =
        Math.abs(AutoAim.getTargetRotation(getPose().getTranslation())
            .minus(getRotation()).getDegrees()) < ledsAlignedMaxDegrees;
    leds.setDriveTargeted(withinRadius && withinRotation);

    // Update brake mode
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        io.setBrakeMode(true);
      }
    } else {
      if (brakeMode
          && Math
              .abs(getLeftVelocityMetersPerSec()) < maxCoastVelocityMetersPerSec
          && Math.abs(
              getRightVelocityMetersPerSec()) < maxCoastVelocityMetersPerSec) {
        brakeMode = false;
        io.setBrakeMode(false);
      }
    }

    // Send tuning constants
    if (kP.hasChanged() | kD.hasChanged()) {
      io.configurePID(kP.get(), 0, kD.get());
    }
  }

  /** Return left position in meters. */
  public double getLeftPositionMeters() {
    if (inputs.externalAvailable && !internalEncoderOverride.get()) {
      return inputs.externalLeftPositionRad * wheelRadiusMeters;
    } else {
      return inputs.leftPositionRad * wheelRadiusMeters;
    }
  }

  /** Return right position in meters. */
  public double getRightPositionMeters() {
    if (inputs.externalAvailable && !internalEncoderOverride.get()) {
      return inputs.externalRightPositionRad * wheelRadiusMeters;
    } else {
      return inputs.rightPositionRad * wheelRadiusMeters;
    }
  }

  /** Return left velocity in meters per second. */
  public double getLeftVelocityMetersPerSec() {
    if (inputs.externalAvailable && !internalEncoderOverride.get()) {
      return inputs.externalRightVelocityRadPerSec * wheelRadiusMeters;
    } else {
      return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
    }
  }

  /** Return right velocity in meters per second. */
  public double getRightVelocityMetersPerSec() {
    if (inputs.externalAvailable && !internalEncoderOverride.get()) {
      return inputs.externalLeftVelocityRadPerSec * wheelRadiusMeters;
    } else {
      return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
    }
  }

  /**
   * Drive at the specified voltage with no other processing. Only use when characterizing.
   */
  public void driveVoltage(double leftVolts, double rightVolts) {
    if (disableOverride.get()) {
      io.setVoltage(0, 0);
      return;
    }

    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * Drive at the specified percentage of max speed.
   */
  public void drivePercent(double leftPercent, double rightPercent) {
    if (disableOverride.get()) {
      io.setVoltage(0, 0);
      return;
    }

    driveVelocity(leftPercent * maxVelocityMetersPerSec,
        rightPercent * maxVelocityMetersPerSec);
  }

  /**
   * Drive at the specified velocity.
   */
  public void driveVelocity(double leftVelocityMetersPerSec,
      double rightVelocityMetersPerSec) {
    if (disableOverride.get()) {
      io.setVoltage(0, 0);
      return;
    }

    // Calculate setpoint and feed forward voltage
    double leftVelocityRadPerSec = leftVelocityMetersPerSec / wheelRadiusMeters;
    double rightVelocityRadPerSec =
        rightVelocityMetersPerSec / wheelRadiusMeters;
    double leftFFVolts = leftModel.calculate(leftVelocityRadPerSec);
    double rightFFVolts = rightModel.calculate(rightVelocityRadPerSec);

    Logger.getInstance().recordOutput("Drive/LeftSetpointRadPerSec",
        leftVelocityRadPerSec);
    Logger.getInstance().recordOutput("Drive/RightSetpointRadPerSec",
        rightVelocityRadPerSec);

    // Send commands to motors
    if (openLoopOverride.get()) {
      // Use open loop control
      io.setVoltage(leftFFVolts, rightFFVolts);
    } else {
      io.setVelocity(leftVelocityRadPerSec, rightVelocityRadPerSec, leftFFVolts,
          rightFFVolts);
    }
  }

  /**
   * In open loop, goes to neutral. In closed loop, resets velocity setpoint.
   */
  public void stop() {
    drivePercent(0, 0);
  }

  /** Resets the pitch measurement. */
  public void resetPitch() {
    basePitchRadians = inputs.gyroPitchPositionRad;
  }

  /** Gets the current pitch measurement in degrees. */
  public double getPitchDegrees() {
    return Math.toDegrees(inputs.gyroPitchPositionRad - basePitchRadians);
  }

  /** Returns the current odometry pose */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the current rotation according to odometry */
  public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose, boolean clearHistory) {
    if (clearHistory) {
      poseHistory = new PoseHistory(poseHistoryCapacity);
    }
    baseDistanceLeftRad = getLeftPositionMeters();
    baseDistanceRightRad = getRightPositionMeters();
    odometry.resetPosition(pose,
        new Rotation2d(inputs.gyroYawPositionRad * -1));
  }

  /** Adds a new timestamped vision measurement */
  public void addVisionMeasurement(TimestampedTranslation2d data) {
    Optional<Pose2d> historicalFieldToTarget = poseHistory.get(data.timestamp);
    if (historicalFieldToTarget.isPresent()) {

      // Get camera constants
      CameraPosition cameraPosition =
          VisionConstants.getCameraPosition(hoodStateSupplier.get());
      if (cameraPosition == null) { // Hood is moving, don't process frame
        return;
      }

      // Calculate new robot pose
      Rotation2d robotRotation = historicalFieldToTarget.get().getRotation();
      Rotation2d cameraRotation =
          robotRotation.rotateBy(cameraPosition.vehicleToCamera.getRotation());
      Transform2d fieldToTargetRotated =
          new Transform2d(FieldConstants.hubCenter, cameraRotation);
      Transform2d fieldToCamera = fieldToTargetRotated.plus(
          GeomUtil.transformFromTranslation(data.translation.unaryMinus()));
      Pose2d visionFieldToTarget = GeomUtil.transformToPose(
          fieldToCamera.plus(cameraPosition.vehicleToCamera.inverse()));

      // Save vision pose for logging
      noVisionTimer.reset();
      lastVisionPose = visionFieldToTarget;

      // Calculate vision percent
      double angularErrorScale =
          Math.abs(inputs.gyroYawVelocityRadPerSec) / visionMaxAngularVelocity;
      angularErrorScale = MathUtil.clamp(angularErrorScale, 0, 1);
      double visionShift =
          1 - Math.pow(1 - visionShiftPerSec, 1 / visionNominalFramerate);
      visionShift *= 1 - angularErrorScale;

      // Reset pose
      Pose2d currentFieldToTarget = getPose();
      Translation2d fieldToVisionField = new Translation2d(
          visionFieldToTarget.getX() - historicalFieldToTarget.get().getX(),
          visionFieldToTarget.getY() - historicalFieldToTarget.get().getY());
      Pose2d visionLatencyCompFieldToTarget =
          new Pose2d(currentFieldToTarget.getX() + fieldToVisionField.getX(),
              currentFieldToTarget.getY() + fieldToVisionField.getY(),
              currentFieldToTarget.getRotation());

      if (resetOnVision) {
        setPose(new Pose2d(visionFieldToTarget.getX(),
            visionFieldToTarget.getY(), currentFieldToTarget.getRotation()),
            true);
        resetOnVision = false;
      } else {
        setPose(new Pose2d(
            currentFieldToTarget.getX() * (1 - visionShift)
                + visionLatencyCompFieldToTarget.getX() * visionShift,
            currentFieldToTarget.getY() * (1 - visionShift)
                + visionLatencyCompFieldToTarget.getY() * visionShift,
            currentFieldToTarget.getRotation()), false);
      }
    }
  }

  /** Reset odometry on the next vision frame (rather than using averaging). */
  public void resetOnNextVision() {
    resetOnVision = true;
  }

  /** Returns whether the vision reset triggered by "resetOnNextVision()" is complete. */
  public boolean getVisionResetComplete() {
    return !resetOnVision;
  }

  /** Return track width in meters. */
  public double getTrackWidthMeters() {
    return trackWidthMeters;
  }

  /** Return average kS. */
  public double getKs() {
    return (leftModel.ks + rightModel.ks) / 2;
  }

  /** Return average kV in (volts * second) / meter. */
  public double getKv() {
    return ((leftModel.kv + rightModel.kv) / 2) / wheelRadiusMeters;
  }

  /** Return average kA in (volts * second^2) / meter. */
  public double getKa() {
    return ((leftModel.ka + rightModel.ka) / 2) / wheelRadiusMeters;
  }

  /** Returns left velocity in radians per second. Only use for characterization. */
  public double getCharacterizationVelocityLeft() {
    return getLeftVelocityMetersPerSec() / wheelRadiusMeters;
  }

  /** Returns right velocity in radians per second. Only use for characterization. */
  public double getCharacterizationVelocityRight() {
    return getRightVelocityMetersPerSec() / wheelRadiusMeters;
  }

  /** Returns gyro position in radians. Only use for characterization */
  public double getCharacterizationGyroPosition() {
    return inputs.gyroYawPositionRad;
  }
}
