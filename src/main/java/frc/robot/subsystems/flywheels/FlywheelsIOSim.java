// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Flywheels subsystem hardware interface for WPILib flywheel sim. */
public class FlywheelsIOSim implements FlywheelsIO {

  private FlywheelSim bigSim =
      new FlywheelSim(DCMotor.getNEO(1), (48.0 / 24.0), 0.004096955);
  private FlywheelSim littleSim = new FlywheelSim(DCMotor.getNEO(1),
      (34.0 / 68.0) * (20.0 / 36.0), 0.000102424);

  private PIDController bigPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController littlePID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double bigFFVolts = 0.0;
  private double littleFFVolts = 0.0;
  private double appliedVoltsBig = 0.0;
  private double appliedVoltsLittle = 0.0;

  public void updateInputs(FlywheelsIOInputs inputs) {
    if (closedLoop) {
      double bigVolts = MathUtil.clamp(
          bigPID.calculate(bigSim.getAngularVelocityRadPerSec()) + bigFFVolts,
          -12.0, 12.0);
      double littleVolts = MathUtil
          .clamp(littlePID.calculate(littleSim.getAngularVelocityRadPerSec())
              + littleFFVolts, -12.0, 12);
      appliedVoltsBig = bigVolts;
      appliedVoltsLittle = littleVolts;
      bigSim.setInputVoltage(appliedVoltsBig);
      littleSim.setInputVoltage(appliedVoltsLittle);

    }

    bigSim.update(Constants.loopPeriodSecs);
    littleSim.update(Constants.loopPeriodSecs);

    inputs.bigPositionRad = 0.0;
    inputs.bigVelocityRadPerSec = bigSim.getAngularVelocityRadPerSec();
    inputs.bigAppliedVolts = appliedVoltsBig;
    inputs.bigCurrentAmps = new double[] {bigSim.getCurrentDrawAmps()};
    inputs.bigTempCelcius = new double[] {};

    inputs.littlePositionRad = 0.0;
    inputs.littleVelocityRadPerSec = littleSim.getAngularVelocityRadPerSec();
    inputs.littleAppliedVolts = appliedVoltsLittle;
    inputs.littleCurrentAmps = new double[] {littleSim.getCurrentDrawAmps()};
    inputs.littleTempCelcius = new double[] {};
  }

  public void setVoltage(double bigVolts, double littleVolts) {
    closedLoop = false;
    appliedVoltsBig = MathUtil.clamp(bigVolts, -12.0, 12.0);
    appliedVoltsLittle = MathUtil.clamp(littleVolts, -12.0, 12.0);
    bigSim.setInputVoltage(appliedVoltsBig);
    littleSim.setInputVoltage(appliedVoltsLittle);
  }

  public void setVelocity(double bigVelocityRadPerSec,
      double littleVelocityRadPerSec, double bigFFVolts, double littleFFVolts) {
    closedLoop = true;
    bigPID.setSetpoint(bigVelocityRadPerSec);
    littlePID.setSetpoint(littleVelocityRadPerSec);
    this.bigFFVolts = bigFFVolts;
    this.littleFFVolts = littleFFVolts;
  }

  public void configurePID(double bigKp, double bigKi, double bigKd,
      double littleKp, double littleKi, double littleKd) {
    bigPID.setP(bigKp);
    bigPID.setI(bigKi);
    bigPID.setD(bigKd);
    littlePID.setP(littleKp);
    littlePID.setI(littleKi);
    littlePID.setD(littleKd);
  }
}
