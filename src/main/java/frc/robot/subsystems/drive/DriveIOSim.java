// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

/** Drive subsystem hardware interface for WPILib drivetrain sim. */
public class DriveIOSim implements DriveIO {

  private static final double wheelRadiusMeters = Units.inchesToMeters(3.0);
  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide,
          KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
  private PIDController leftPID =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private PIDController rightPID =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private boolean closedLoop = false;
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private double appliedVoltsLeft = 0.0;
  private double appliedVoltsRight = 0.0;

  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      double leftVolts = leftPID
          .calculate(sim.getLeftVelocityMetersPerSecond() / wheelRadiusMeters)
          + leftFFVolts;
      double rightVolts = rightPID
          .calculate(sim.getRightVelocityMetersPerSecond() / wheelRadiusMeters)
          + rightFFVolts;
      appliedVoltsLeft = leftVolts;
      appliedVoltsRight = rightVolts;
      sim.setInputs(leftVolts, rightVolts);
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.leftPositionRad = sim.getLeftPositionMeters() / wheelRadiusMeters;
    inputs.leftVelocityRadPerSec =
        sim.getLeftVelocityMetersPerSecond() / wheelRadiusMeters;
    inputs.leftAppliedVolts = appliedVoltsLeft;
    inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};
    inputs.leftTempCelcius = new double[] {};

    inputs.rightPositionRad = sim.getRightPositionMeters() / wheelRadiusMeters;
    inputs.rightVelocityRadPerSec =
        sim.getRightVelocityMetersPerSecond() / wheelRadiusMeters;
    inputs.rightAppliedVolts = appliedVoltsRight;
    inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};
    inputs.rightTempCelcius = new double[] {};

    double lastGyroPosition = inputs.gyroYawPositionRad;
    inputs.gyroYawPositionRad = sim.getHeading().getRadians() * -1;
    inputs.gyroYawVelocityRadPerSec =
        (inputs.gyroYawPositionRad - lastGyroPosition)
            / Constants.loopPeriodSecs;
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    appliedVoltsLeft = leftVolts;
    appliedVoltsRight = rightVolts;
    sim.setInputs(leftVolts, rightVolts);
  }

  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftVelocityRadPerSec);
    rightPID.setSetpoint(rightVelocityRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
  }

  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);
    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }
}
