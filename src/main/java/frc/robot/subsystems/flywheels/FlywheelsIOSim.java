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

  private FlywheelSim sim =
      new FlywheelSim(DCMotor.getNEO(1), (48.0 / 24.0), 0.004096955);

  private PIDController pid =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public void updateInputs(FlywheelsIOInputs inputs) {
    if (closedLoop) {
      double bigVolts = MathUtil.clamp(
          pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0,
          12.0);
      appliedVolts = bigVolts;
      sim.setInputVoltage(appliedVolts);

    }

    sim.update(Constants.loopPeriodSecs);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {};
  }

  public void setVoltage(double bigVolts, double littleVolts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(bigVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  public void setVelocity(double bigVelocityRadPerSec,
      double littleVelocityRadPerSec, double bigFFVolts, double littleFFVolts) {
    closedLoop = true;
    pid.setSetpoint(bigVelocityRadPerSec);
    this.ffVolts = bigFFVolts;
  }

  public void configurePID(double bigKp, double bigKi, double bigKd,
      double littleKp, double littleKi, double littleKd) {
    pid.setP(bigKp);
    pid.setI(bigKi);
    pid.setD(bigKd);
  }
}
