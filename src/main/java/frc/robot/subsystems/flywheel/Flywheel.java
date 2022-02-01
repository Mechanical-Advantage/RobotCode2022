// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SysIdCommand.MechanismSysIdData;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.util.TunableNumber;
import frc.robot.util.VelocityProfiler;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();

  private final SimpleMotorFeedforward bigFFModel;
  private final SimpleMotorFeedforward littleFFModel;
  private final TunableNumber bigKp = new TunableNumber("Flywheel/BigKp");
  private final TunableNumber bigKd = new TunableNumber("Flywheel/BigKd");
  private final TunableNumber littleKp = new TunableNumber("Flywheel/LittleKp");
  private final TunableNumber littleKd = new TunableNumber("Flywheel/LittleKd");
  private final TunableNumber bigToleranceRpm =
      new TunableNumber("Flywheel/BigToleranceRPM");
  private final TunableNumber littleToleranceRpm =
      new TunableNumber("Flywheel/LittleToleranceRPM");

  private final VelocityProfiler bigProfiler = new VelocityProfiler(1000.0);
  private final VelocityProfiler littleProfiler = new VelocityProfiler(1000.0);
  private boolean closedLoop = false;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        bigFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        bigKp.setDefault(0.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(0.0);

        littleFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        littleKp.setDefault(0.0);
        littleKd.setDefault(0.0);
        littleToleranceRpm.setDefault(0.0);
        break;
      default:
        bigFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        bigKp.setDefault(0.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(0.0);

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
    Logger.getInstance().processInputs("Flywheel", inputs);

    if (bigKp.hasChanged() | bigKd.hasChanged() | littleKp.hasChanged()
        | littleKd.hasChanged()) {
      io.configurePID(bigKp.get(), 0.0, bigKd.get(), littleKp.get(), 0.0,
          littleKd.get());
    }

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

  /** Run at the specified voltage with no other processing. Only use with SysId. */
  public void runVoltage(double bigVolts, double littleVolts) {
    io.setVoltage(bigVolts, littleVolts);
    closedLoop = false;
    bigProfiler.reset();
    littleProfiler.reset();
  }

  /** Run at velocity with closed loop control. */
  public void runVelocity(double bigRPM, double littleRPM) {
    if (closedLoop) {
      bigProfiler.setSetpointGoal(bigRPM);
      littleProfiler.setSetpointGoal(littleRPM);
    } else {
      bigProfiler.setSetpointGoal(bigRPM, getBigVelocity());
      littleProfiler.setSetpointGoal(littleRPM, getLittleVelocity());
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

  /** Returns whether the velocity has reached the closed loop setpoint. */
  public boolean atSetpoint() {
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

  /**
   * Returns a set of data for SysId (big flywheel).
   */
  public MechanismSysIdData getBigSysIdData() {
    return new MechanismSysIdData(inputs.bigPositionRad,
        inputs.bigVelocityRadPerSec);
  }

  /**
   * Returns a set of data for SysId (little flywheel).
   */
  public MechanismSysIdData getLittleSysIdData() {
    return new MechanismSysIdData(inputs.littlePositionRad,
        inputs.littleVelocityRadPerSec);
  }
}
