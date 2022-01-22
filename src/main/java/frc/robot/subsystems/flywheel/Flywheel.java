// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.util.TunableNumber;
import frc.robot.util.VelocityProfiler;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();

  private final SimpleMotorFeedforward ffModel;
  private final TunableNumber kP = new TunableNumber("Flywheel/kP");
  private final TunableNumber kD = new TunableNumber("Flywheel/kD");
  private final TunableNumber toleranceRpm =
      new TunableNumber("Flywheel/ToleranceRPM");

  private final VelocityProfiler profiler = new VelocityProfiler(1000.0);
  private boolean closedLoop = false;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        kP.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRpm.setDefault(300.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        kP.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRpm.setDefault(0.0);
    }

    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Flywheel", inputs);

    if (kP.hasChanged() | kD.hasChanged()) {
      io.configurePID(kP.get(), 0.0, kD.get());
    }

    if (closedLoop) {
      double velocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(profiler.getSetpoint());
      io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    }
  }

  /** Run at the specified voltage with no other processing. Only use with SysId. */
  public void runVoltage(double volts) {
    io.setVoltage(volts);
    closedLoop = false;
    profiler.reset();
  }

  /** Run at velocity with closed loop control. */
  public void runVelocity(double rpm) {
    if (closedLoop) {
      profiler.setSetpointGoal(rpm);
    } else {
      profiler.setSetpointGoal(rpm, getVelocity());
    }
    closedLoop = true;
  }

  /** Stops by going to open loop. */
  public void stop() {
    runVoltage(0.0);
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns whether the velocity has reached the closed loop setpoint. */
  public boolean atSetpoint() {
    if (closedLoop) {
      return Math.abs(getVelocity() - profiler.getSetpointGoal()) < toleranceRpm
          .get();
    } else {
      return false;
    }
  }
}
