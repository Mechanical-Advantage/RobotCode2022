// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private final SimpleMotorFeedforward leftModel, rightModel;
  private final TunableNumber kP = new TunableNumber("Drive/kP");
  private final TunableNumber kD = new TunableNumber("Drive/kD");

  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();

  private Supplier<Boolean> disableOverride = () -> false;
  private Supplier<Boolean> openLoopOverride = () -> false;

  private boolean brakeMode = false;

  /** Creates a new DriveTrain. */
  public Drive(DriveIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2020:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        leftModel = new SimpleMotorFeedforward(0.23004, 0.2126, 0.036742);
        rightModel = new SimpleMotorFeedforward(0.22652, 0.21748, 0.03177);
        kP.setDefault(0.00015);
        kD.setDefault(0.0015);
        break;
      case ROBOT_KITBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(122.0);
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        leftModel = new SimpleMotorFeedforward(0, 0, 0);
        rightModel = new SimpleMotorFeedforward(0, 0, 0);
        kP.setDefault(2);
        kD.setDefault(40);
        break;
      default:
        maxVelocityMetersPerSec = 0;
        wheelRadiusMeters = Double.POSITIVE_INFINITY;
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

  /** Return left velocity in meters per second. */
  private double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Return right velocity in meters per second. */
  private double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
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
