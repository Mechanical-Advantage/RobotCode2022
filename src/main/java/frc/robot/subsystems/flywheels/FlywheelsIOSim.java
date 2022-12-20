// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Flywheels subsystem hardware interface for WPILib flywheel sim. */
public class FlywheelsIOSim implements FlywheelsIO {

  private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), (48.0 / 24.0), 0.004096955);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public void updateInputs(FlywheelsIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {};
  }

  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
