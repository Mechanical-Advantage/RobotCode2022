// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SysIdCommand.DriveTrainSysIdData;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.util.TunableNumber;

public class Drive extends SubsystemBase {
  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
                                                                   // switch to coast when disabling

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

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d());
  private boolean brakeMode = false;

  /** Creates a new DriveTrain. */
  public Drive(DriveIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2020:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        trackWidthMeters = 1.768748;
        leftModel = new SimpleMotorFeedforward(0.23004, 0.2126, 0.036742);
        rightModel = new SimpleMotorFeedforward(0.22652, 0.21748, 0.03177);
        kP.setDefault(0.00015);
        kD.setDefault(0.0015);
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

  /** Set boolean supplier for the override switches. */
  public void setOverrides(Supplier<Boolean> disableOverride,
      Supplier<Boolean> openLoopOverride) {
    this.disableOverride = disableOverride;
    this.openLoopOverride = openLoopOverride;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    // Update odometry
    odometry.update(new Rotation2d(inputs.gyroPositionRad * -1),
        inputs.leftPositionRad * wheelRadiusMeters,
        inputs.rightPositionRad * wheelRadiusMeters);
    Logger.getInstance().recordOutput("Odometry/Robot",
        new double[] {odometry.getPoseMeters().getX(),
            odometry.getPoseMeters().getY(),
            odometry.getPoseMeters().getRotation().getRadians()});

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

  /**
   * Drive at the specified voltage with no other processing. Only use with SysId.
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


    double leftVelocityRadPerSec = leftVelocityMetersPerSec / wheelRadiusMeters;
    double rightVelocityRadPerSec =
        rightVelocityMetersPerSec / wheelRadiusMeters;

    double leftFFVolts = leftModel.calculate(leftVelocityRadPerSec);
    double rightFFVolts = rightModel.calculate(rightVelocityRadPerSec);

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

  /** Returns the current odometry pose */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the current rotation according to odometry */
  public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    io.resetPosition(0, 0);
    odometry.resetPosition(pose, new Rotation2d(inputs.gyroPositionRad * -1));
  }

  /** Return left velocity in meters per second. */
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Return right velocity in meters per second. */
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
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

  /**
   * Returns a set of data for SysId
   */
  public DriveTrainSysIdData getSysIdData() {
    return new DriveTrainSysIdData(inputs.leftPositionRad,
        inputs.rightPositionRad, inputs.leftVelocityRadPerSec,
        inputs.rightVelocityRadPerSec, inputs.gyroPositionRad,
        inputs.gyroVelocityRadPerSec);
  }
}
