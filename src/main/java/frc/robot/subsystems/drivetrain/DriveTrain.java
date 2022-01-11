// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SysIdCommand.DriveTrainSysIdData;
import frc.robot.subsystems.drivetrain.DriveTrainIO.DriveTrainIOInputs;

public class DriveTrain extends SubsystemBase {
  private final double wheelRadiusM;
  private final SimpleMotorFeedforward leftModel, rightModel;

  private final DriveTrainIO io;
  private final DriveTrainIOInputs inputs = new DriveTrainIOInputs();

  private Supplier<Boolean> disableOverride = () -> false;
  private Supplier<Boolean> openLoopOverride = () -> false;;

  /** Creates a new DriveTrain. */
  public DriveTrain(DriveTrainIO io) {
    this.io = io;

    switch (Constants.getRobot()) {
      // case ROBOT_2022C:
      // break;
      case ROBOT_2020:
        wheelRadiusM = Units.inchesToMeters(3.0);
        leftModel = new SimpleMotorFeedforward(0, 0, 0);
        rightModel = new SimpleMotorFeedforward(0, 0, 0);
        break;
      // case ROBOT_2022P:
      // break;
      // case ROBOT_KITBOT:
      // break;
      // case ROBOT_ROMI:
      // break;
      // case ROBOT_SIMBOT:
      // break;
      default:
        wheelRadiusM = Double.POSITIVE_INFINITY;
        leftModel = new SimpleMotorFeedforward(0, 0, 0);
        rightModel = new SimpleMotorFeedforward(0, 0, 0);

        break;

    }
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
    Logger.getInstance().processInputs("DriveTrain", inputs);
  }

  /**
   * Drive at the specified voltage with no other processing. Only use with SysId.
   */
  public void driveVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * Drive at the specified percentage of max speed.
   */
  public void drivePercent(double leftPercent, double rightPercent) {
    io.setVoltage(leftPercent * 12.0, rightPercent * 12.0);
  }

  /**
   * Drive at the specified velocity.
   */
  public void driveVelocity(double leftVelocityMetersPerSec,
      double rightVelocityMetersPerSec) {
    double leftVelocityRadPerSec = leftVelocityMetersPerSec / wheelRadiusM;
    double rightVelocityRadPerSec = rightVelocityMetersPerSec / wheelRadiusM;

    double leftFFVolts = leftModel.calculate(leftVelocityRadPerSec);
    double rightFFVolts = rightModel.calculate(rightVelocityRadPerSec);

    if (openLoopOverride.get()) {
      // Use open loop control
      io.setVoltage(leftFFVolts, rightFFVolts);
    } else {
      io.setVelocity(leftVelocityRadPerSec,
          rightVelocityRadPerSec / wheelRadiusM, leftFFVolts, rightFFVolts);
    }
  }

  /**
   * In open loop, goes to neutral. In closed loop, resets velocity setpoint.
   */
  public void stop() {
    driveVelocity(0, 0);
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
