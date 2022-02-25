// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.flywheels.FlywheelsIO.FlywheelsIOInputs;
import frc.robot.util.TunableNumber;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputs inputs = new FlywheelsIOInputs();

  private final TunableNumber rpmHistoryLength =
      new TunableNumber("Flywheels/RPMHistoryLength");
  private final TunableNumber bigVelocityRpm =
      new TunableNumber("Flywheels/BigVelocityRPM");
  private final TunableNumber littleVelocityRpm =
      new TunableNumber("Flywheels/LittleVelocityRPM");
  private final TunableNumber bigAccelerationRpmPerSec2 =
      new TunableNumber("Flywheels/BigAccelerationRPMPerSec2");
  private final TunableNumber littleAccelerationRpmPerSec2 =
      new TunableNumber("Flywheels/LittleAccelerationRPMPerSec2");
  private final TunableNumber bigJerkRpmPerSec3 =
      new TunableNumber("Flywheels/BigJerkRPMPerSec3");
  private final TunableNumber littleJerkRpmPerSec3 =
      new TunableNumber("Flywheels/LittleJerkRPMPerSec3");
  private final SimpleMotorFeedforward bigFFModel;
  private final SimpleMotorFeedforward littleFFModel;
  private final TunableNumber bigKp = new TunableNumber("Flywheels/BigKp");
  private final TunableNumber bigKi = new TunableNumber("Flywheels/BigKi");
  private final TunableNumber bigKd = new TunableNumber("Flywheels/BigKd");
  private final TunableNumber littleKp =
      new TunableNumber("Flywheels/LittleKp");
  private final TunableNumber littleKi =
      new TunableNumber("Flywheels/LittleKi");
  private final TunableNumber littleKd =
      new TunableNumber("Flywheels/LittleKd");
  private final TunableNumber bigToleranceRpm =
      new TunableNumber("Flywheels/BigToleranceRPM");
  private final TunableNumber littleToleranceRpm =
      new TunableNumber("Flywheels/LittleToleranceRPM");

  private boolean closedLoop = false;
  private List<Double> bigRpmHistory = new ArrayList<>();
  private List<Double> littleRpmHistory = new ArrayList<>();
  private TrapezoidProfile.State bigGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State littleGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State bigLastState = new TrapezoidProfile.State();
  private TrapezoidProfile.State littleLastState = new TrapezoidProfile.State();
  private boolean bigProfileComplete = false;
  private boolean littleProfileComplete = false;

  /** Creates a new Flywheels. */
  public Flywheels(FlywheelsIO io) {
    this.io = io;

    rpmHistoryLength.setDefault(10);
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        bigVelocityRpm.setDefault(2650.0);
        bigAccelerationRpmPerSec2.setDefault(2000.0);
        bigJerkRpmPerSec3.setDefault(2500.0);
        bigFFModel = new SimpleMotorFeedforward(0.41440, 0.0425);
        bigKp.setDefault(0.00006);
        bigKi.setDefault(0.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(50.0);

        littleVelocityRpm.setDefault(9500.0);
        littleAccelerationRpmPerSec2.setDefault(4000.0);
        littleJerkRpmPerSec3.setDefault(6000.0);
        littleFFModel = new SimpleMotorFeedforward(0.41783, 0.0105);
        littleKp.setDefault(0.0003);
        littleKi.setDefault(0.0);
        littleKd.setDefault(0.0);
        littleToleranceRpm.setDefault(100.0);
        break;
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        bigVelocityRpm.setDefault(2800.0);
        bigAccelerationRpmPerSec2.setDefault(8000.0);
        bigJerkRpmPerSec3.setDefault(9999.9);
        bigFFModel = new SimpleMotorFeedforward(0.0574, 0.03979);
        bigKp.setDefault(0.6);
        bigKi.setDefault(0.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(50.0);

        littleVelocityRpm.setDefault(20700.0);
        littleAccelerationRpmPerSec2.setDefault(20000.0);
        littleJerkRpmPerSec3.setDefault(9999.0);
        littleFFModel = new SimpleMotorFeedforward(0.06896, 0.00556);
        littleKp.setDefault(0.06);
        littleKi.setDefault(0.0);
        littleKd.setDefault(0.0);
        littleToleranceRpm.setDefault(50.0);
        break;
      default:
        bigVelocityRpm.setDefault(0.0);
        bigAccelerationRpmPerSec2.setDefault(0.0);
        bigJerkRpmPerSec3.setDefault(0.0);
        bigFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        bigKp.setDefault(0.0);
        bigKi.setDefault(0.0);
        bigKd.setDefault(0.0);
        bigToleranceRpm.setDefault(0.0);

        littleVelocityRpm.setDefault(0.0);
        littleAccelerationRpmPerSec2.setDefault(0.0);
        littleJerkRpmPerSec3.setDefault(0.0);
        littleFFModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        littleKp.setDefault(0.0);
        littleKi.setDefault(0.0);
        littleKd.setDefault(0.0);
        littleToleranceRpm.setDefault(0.0);
    }

    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Flywheels", inputs);

    if (bigKp.hasChanged() | bigKi.hasChanged() | bigKd.hasChanged()
        | littleKp.hasChanged() | littleKi.hasChanged()
        | littleKd.hasChanged()) {
      io.configurePID(bigKp.get(), bigKi.get(), bigKd.get(), littleKp.get(),
          littleKi.get(), littleKd.get());
    }

    bigRpmHistory.add(getBigVelocity());
    littleRpmHistory.add(getLittleVelocity());
    while (bigRpmHistory.size() > rpmHistoryLength.get()) {
      bigRpmHistory.remove(0);
    }
    while (littleRpmHistory.size() > rpmHistoryLength.get()) {
      littleRpmHistory.remove(0);
    }

    Logger.getInstance().recordOutput("Flywheels/BigRPM", getBigVelocity());
    Logger.getInstance().recordOutput("Flywheels/LittleRPM",
        getLittleVelocity());
    Logger.getInstance().recordOutput("Flywheels/BigAcceleration",
        getBigAcceleration());
    Logger.getInstance().recordOutput("Flywheels/LittleAcceleration",
        getLittleAcceleration());
    Logger.getInstance().recordOutput("Flywheels/AtSetpoints", atSetpoints());

    if (closedLoop) {
      TrapezoidProfile bigProfile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(bigAccelerationRpmPerSec2.get(),
              bigJerkRpmPerSec3.get()),
          bigGoal, bigLastState);
      bigLastState = bigProfile.calculate(Constants.loopPeriodSecs);
      double bigSetpointRpm = bigLastState.position;

      TrapezoidProfile littleProfile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(littleAccelerationRpmPerSec2.get(),
              littleJerkRpmPerSec3.get()),
          littleGoal, littleLastState);
      littleLastState = littleProfile.calculate(Constants.loopPeriodSecs);
      double littleSetpointRpm = littleLastState.position;

      bigProfileComplete = bigSetpointRpm == bigGoal.position;
      littleProfileComplete = littleSetpointRpm == littleGoal.position;
      Logger.getInstance().recordOutput("Flywheels/BigSetpointRPM",
          bigSetpointRpm);
      Logger.getInstance().recordOutput("Flywheels/LittleSetpointRPM",
          littleSetpointRpm);
      Logger.getInstance().recordOutput("Flywheels/BigProfileComplete",
          bigProfileComplete);
      Logger.getInstance().recordOutput("Flywheels/LittleProfileComplete",
          littleProfileComplete);

      double bigVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(bigSetpointRpm);
      double littleVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(littleSetpointRpm);
      io.setVelocity(bigVelocityRadPerSec, littleVelocityRadPerSec,
          bigFFModel.calculate(bigVelocityRadPerSec),
          littleFFModel.calculate(littleVelocityRadPerSec));
    } else {
      bigProfileComplete = false;
      littleProfileComplete = false;
    }
  }

  /** Run at the specified voltage with no other processing. Only use when characterizing. */
  public void runVoltage(double bigVolts, double littleVolts) {
    io.setVoltage(bigVolts, littleVolts);
    closedLoop = false;
  }

  /** Run at velocity with closed loop control. */
  public void runVelocity(double bigRpm, double littleRpm) {
    bigRpm =
        MathUtil.clamp(bigRpm, -bigVelocityRpm.get(), bigVelocityRpm.get());
    littleRpm = MathUtil.clamp(littleRpm, -littleVelocityRpm.get(),
        littleVelocityRpm.get());
    bigGoal = new TrapezoidProfile.State(bigRpm, 0.0);
    littleGoal = new TrapezoidProfile.State(littleRpm, 0.0);
    if (!closedLoop) {
      bigLastState =
          new TrapezoidProfile.State(getBigVelocity(), getBigAcceleration());

      // The little flywheel spins down so quickly that using the current acceleration causes
      // undesirable behavior.
      littleLastState = new TrapezoidProfile.State(getLittleVelocity(), 0.0);
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

  /** Returns the current acceleration of the big flywheel in RPM per second. */
  public double getBigAcceleration() {
    return (getBigVelocity() - bigRpmHistory.get(0))
        / (Constants.loopPeriodSecs * rpmHistoryLength.get());
  }

  /** Returns the current acceleration of the little flywheel in RPM per second. */
  public double getLittleAcceleration() {
    return (getLittleVelocity() - littleRpmHistory.get(0))
        / (Constants.loopPeriodSecs * rpmHistoryLength.get());
  }

  /** Returns whether the velocity has reached the closed loop setpoint on both flywheels. */
  public boolean atSetpoints() {
    if (closedLoop) {
      boolean bigAtSetpoint =
          Math.abs(getBigVelocity() - bigGoal.position) < bigToleranceRpm.get();
      boolean littleAtSetpoint = Math.abs(
          getLittleVelocity() - littleGoal.position) < littleToleranceRpm.get();
      return bigAtSetpoint && littleAtSetpoint;
    } else {
      return false;
    }
  }

  /** Returns whether the velocity setpoints have reached the goals. */
  public boolean profilesComplete() {
    return bigProfileComplete && littleProfileComplete;
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
