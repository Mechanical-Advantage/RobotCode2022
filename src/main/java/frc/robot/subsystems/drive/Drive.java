// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.Alert;
import frc.robot.util.TunableNumber;
import frc.robot.util.Alert.AlertType;

public class Drive extends SubsystemBase {
  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
                                                                   // switch to coast when disabling
  private static final double ledsClimbFailureAccelMetersPerSec2 = 40.0; // Threshold to detect
                                                                         // climb failures
  private static final double ledsFallenAngleDegrees = 75.0; // Threshold to detect falls

  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro sensor disconnected, odometry will be very inaccurate",
          AlertType.ERROR);

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
  private RobotState robotState;
  private Leds leds;

  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;
  private boolean lastGyroConnected = false;
  private Rotation2d lastGyroRotation = new Rotation2d();
  private boolean brakeMode = false;
  private boolean pitchResetComplete = false;
  private double basePitchRadians = 0.0;

  /** Creates a new DriveTrain. */
  public Drive(DriveIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        wheelRadiusMeters = Units.inchesToMeters(2.0849755);
        trackWidthMeters = 0.65778;
        leftModel = new SimpleMotorFeedforward(0.23071, 0.12413);
        rightModel = new SimpleMotorFeedforward(0.23455, 0.12270);
        kP.setDefault(0.00008);
        kD.setDefault(0.0);
        break;
      case ROBOT_2022P:
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        wheelRadiusMeters = Units.inchesToMeters(2.0);
        trackWidthMeters = 0.78383;
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
  }

  /** Set suppliers for external data. */
  public void setSuppliers(Supplier<Boolean> disableOverride,
      Supplier<Boolean> openLoopOverride,
      Supplier<Boolean> internalEncoderOverride) {
    this.disableOverride = disableOverride;
    this.openLoopOverride = openLoopOverride;
    this.internalEncoderOverride = internalEncoderOverride;
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
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
    Rotation2d currentGyroRotation =
        new Rotation2d(inputs.gyroYawPositionRad * -1);
    double leftPositionMetersDelta =
        getLeftPositionMeters() - lastLeftPositionMeters;
    double rightPositionMetersDelta =
        getRightPositionMeters() - lastRightPositionMeters;
    double avgPositionMetersDelta =
        (leftPositionMetersDelta + rightPositionMetersDelta) / 2.0;
    Rotation2d gyroRotationDelta =
        (inputs.gyroConnected && !lastGyroConnected) ? new Rotation2d()
            : currentGyroRotation.minus(lastGyroRotation);

    if (inputs.gyroConnected) {
      robotState.addDriveData(Timer.getFPGATimestamp(), new Twist2d(
          avgPositionMetersDelta, 0.0, gyroRotationDelta.getRadians()));
    } else {
      robotState.addDriveData(Timer.getFPGATimestamp(),
          new Twist2d(avgPositionMetersDelta, 0.0,
              (rightPositionMetersDelta - leftPositionMetersDelta)
                  / trackWidthMeters));
    }

    lastLeftPositionMeters = getLeftPositionMeters();
    lastRightPositionMeters = getRightPositionMeters();
    lastGyroConnected = inputs.gyroConnected;
    lastGyroRotation = currentGyroRotation;

    // Update gyro alert
    gyroDisconnectedAlert.set(!inputs.gyroConnected);

    // Manage pitch rotation
    if (!pitchResetComplete) {
      resetPitch();
      pitchResetComplete = true;
    }
    Logger.getInstance().recordOutput("Drive/PitchDegrees", getPitchDegrees());

    // Check for climb failure (high z acceleration)
    if (Math.abs(
        inputs.gyroZAccelMetersPerSec2) > ledsClimbFailureAccelMetersPerSec2) {
      leds.setClimbFailure(true);
    }

    // Check for fallen robot
    leds.setFallen(Units.radiansToDegrees(
        Math.abs(inputs.gyroPitchPositionRad)) > ledsFallenAngleDegrees
        || Units.radiansToDegrees(
            Math.abs(inputs.gyroRollPositionRad)) > ledsFallenAngleDegrees);

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
