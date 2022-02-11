// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.flywheels.FlywheelsIO.FlywheelsIOInputs;
import frc.robot.util.TunableNumber;
import frc.robot.util.VelocityProfiler;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputs inputs = new FlywheelsIOInputs();

  private final TunableNumber bigMaxVelocityRpm =
      new TunableNumber("Flywheels/BigMaxVelocityRPM");
  private final TunableNumber littleMaxVelocityRpm =
      new TunableNumber("Flywheels/LittleMaxVelocityRPM");
  private final SimpleMotorFeedforward bigFFModel;
  private final SimpleMotorFeedforward littleFFModel;
  private final TunableNumber bigKp = new TunableNumber("Flywheels/BigKp");
  private final TunableNumber bigKd = new TunableNumber("Flywheels/BigKd");
  private final TunableNumber littleKp =
      new TunableNumber("Flywheels/LittleKp");
  private final TunableNumber littleKd =
      new TunableNumber("Flywheels/LittleKd");
  private final TunableNumber bigToleranceRpm =
      new TunableNumber("Flywheels/BigToleranceRPM");
  private final TunableNumber littleToleranceRpm =
      new TunableNumber("Flywheels/LittleToleranceRPM");

  private final VelocityProfiler bigProfiler = new VelocityProfiler(1000.0);
  private final VelocityProfiler littleProfiler = new VelocityProfiler(1000.0);
  private boolean closedLoop = false;

  /** Creates a new Flywheels. */
  public Flywheels(FlywheelsIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        bigMaxVelocityRpm.setDefault(3800.0);
        bigFFModel = new SimpleMotorFeedforward(0.0, 0.028214, 0.094281);
        bigKp.setDefault(5.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(50.0);

        littleMaxVelocityRpm.setDefault(3800.0);
        littleFFModel = new SimpleMotorFeedforward(0.0, 0.029752, 0.015384);
        littleKp.setDefault(1.0);
        littleKd.setDefault(0.0);
        littleToleranceRpm.setDefault(50.0);
        break;
      default:
        bigMaxVelocityRpm.setDefault(0.0);
        bigFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        bigKp.setDefault(0.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(0.0);

        littleMaxVelocityRpm.setDefault(0.0);
        littleFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        littleKp.setDefault(0.0);
        littleKd.setDefault(0.0);
        littleToleranceRpm.setDefault(0.0);
    }

    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Flywheels", inputs);

    if (bigKp.hasChanged() | bigKd.hasChanged() | littleKp.hasChanged()
        | littleKd.hasChanged()) {
      io.configurePID(bigKp.get(), 0.0, bigKd.get(), littleKp.get(), 0.0,
          littleKd.get());
    }

    Logger.getInstance().recordOutput("Flywheels/BigRPM", Units
        .radiansPerSecondToRotationsPerMinute(inputs.bigVelocityRadPerSec));
    Logger.getInstance().recordOutput("Flywheels/LittleRPM", Units
        .radiansPerSecondToRotationsPerMinute(inputs.littleVelocityRadPerSec));

    if (closedLoop) {
      double bigVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(bigProfiler.getSetpoint());
      double littleVelocityRadPerSec = Units
          .rotationsPerMinuteToRadiansPerSecond(littleProfiler.getSetpoint());
      io.setVelocity(bigVelocityRadPerSec, littleVelocityRadPerSec,
          bigFFModel.calculate(bigVelocityRadPerSec),
          littleFFModel.calculate(littleVelocityRadPerSec));
    }
  }

  /** Run at the specified voltage with no other processing. Only use when characterizing. */
  public void runVoltage(double bigVolts, double littleVolts) {
    io.setVoltage(bigVolts, littleVolts);
    closedLoop = false;
    bigProfiler.reset();
    littleProfiler.reset();
  }

  /** Run at velocity with closed loop control. */
  public void runVelocity(double bigRpm, double littleRpm) {
    bigRpm = MathUtil.clamp(bigRpm, -bigMaxVelocityRpm.get(),
        bigMaxVelocityRpm.get());
    littleRpm = MathUtil.clamp(littleRpm, -littleMaxVelocityRpm.get(),
        littleMaxVelocityRpm.get());
    if (closedLoop) {
      bigProfiler.setSetpointGoal(bigRpm);
      littleProfiler.setSetpointGoal(littleRpm);
    } else {
      bigProfiler.setSetpointGoal(bigRpm, getBigVelocity());
      littleProfiler.setSetpointGoal(littleRpm, getLittleVelocity());
    }
    closedLoop = true;
  }

  /** Stops by going to open loop. */
  public void stop() {
    runVoltage(0.0, 0.0);
  }

  /** Returns the current velocity of the big flywheel in RPM. */
  public double getBigVelocity() {
    return Units
        .radiansPerSecondToRotationsPerMinute(inputs.bigVelocityRadPerSec);
  }

  /** Returns the current velocity of the little flywheel in RPM. */
  public double getLittleVelocity() {
    return Units
        .radiansPerSecondToRotationsPerMinute(inputs.littleVelocityRadPerSec);
  }

  /** Returns whether the velocity has reached the closed loop setpoint on both flywheels. */
  public boolean atSetpoints() {
    if (closedLoop) {
      boolean bigAtSetpoint = Math.abs(getBigVelocity()
          - bigProfiler.getSetpointGoal()) < bigToleranceRpm.get();
      boolean littleAtSetpoint = Math.abs(getLittleVelocity()
          - littleProfiler.getSetpointGoal()) < littleToleranceRpm.get();
      return bigAtSetpoint && littleAtSetpoint;
    } else {
      return false;
    }
  }

  /** Returns velocity of big flywheel in radians per second. Only use for characterization. */
  public double getCharacterizationVelocityBig() {
    return inputs.bigVelocityRadPerSec;
  }

  /** Returns velocity of little flywheel in radians per second. Only use for characterization. */
  public double getCharacterizationVelocityLittle() {
    return inputs.littleVelocityRadPerSec;
  }
}
