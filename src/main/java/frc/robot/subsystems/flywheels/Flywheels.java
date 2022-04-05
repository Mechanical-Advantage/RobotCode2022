// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.flywheels.FlywheelsIO.FlywheelsIOInputs;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.TunableNumber;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputs inputs = new FlywheelsIOInputs();

  private final TunableNumber rpmHistoryLength =
      new TunableNumber("Flywheels/RPMHistoryLength");
  private final TunableNumber maxVelocityRpm =
      new TunableNumber("Flywheels/MaxVelocityRPM");
  private final TunableNumber maxAccelerationRpmPerSec2 =
      new TunableNumber("Flywheels/MaxAccelerationRPMPerSec2");
  private final TunableNumber maxJerkRpmPerSec3 =
      new TunableNumber("Flywheels/MaxJerkRPMPerSec3");
  private final SimpleMotorFeedforward ffModel;
  private final TunableNumber kP = new TunableNumber("Flywheels/kP");
  private final TunableNumber kI = new TunableNumber("Flywheels/kI");
  private final TunableNumber kD = new TunableNumber("Flywheels/kD");
  private final TunableNumber toleranceRpm =
      new TunableNumber("Flywheels/ToleranceRPM");

  private boolean closedLoop = false;
  private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private double offsetRpm = 0;
  private Leds leds;

  /** Creates a new Flywheels. */
  public Flywheels(FlywheelsIO io) {
    this.io = io;

    rpmHistoryLength.setDefault(10);
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        maxVelocityRpm.setDefault(2650.0);
        maxAccelerationRpmPerSec2.setDefault(2000.0);
        maxJerkRpmPerSec3.setDefault(2500.0);
        ffModel = new SimpleMotorFeedforward(0.50343, 0.04222);
        kP.setDefault(0.000111);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRpm.setDefault(50.0);
        offsetRpm = 0.0;
        break;
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        maxVelocityRpm.setDefault(2800.0);
        maxAccelerationRpmPerSec2.setDefault(8000.0);
        maxJerkRpmPerSec3.setDefault(9999.9);
        ffModel = new SimpleMotorFeedforward(0.0574, 0.03979);
        kP.setDefault(0.6);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRpm.setDefault(50.0);
        offsetRpm = 0.0;
        break;
      default:
        maxVelocityRpm.setDefault(0.0);
        maxAccelerationRpmPerSec2.setDefault(0.0);
        maxJerkRpmPerSec3.setDefault(0.0);
        ffModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        kP.setDefault(0.0);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRpm.setDefault(0.0);
        offsetRpm = 0.0;
    }

    io.setBrakeMode(false);
  }

  public void setLeds(Leds leds) {
    this.leds = leds;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Flywheels", inputs);

    // Update tuning constants
    if (kP.hasChanged() | kI.hasChanged() | kD.hasChanged()) {
      io.configurePID(kP.get(), kI.get(), kD.get());
    }

    // Log data
    Logger.getInstance().recordOutput("Flywheels/RPM", getVelocity());
    Logger.getInstance().recordOutput("Flywheels/OffsetRPM", offsetRpm);
    SmartDashboard.putNumber("Flywheel Offset", offsetRpm);

    // Set closed loop setpoint
    if (closedLoop) {
      TrapezoidProfile profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxAccelerationRpmPerSec2.get(),
              maxJerkRpmPerSec3.get()),
          profileGoal, profileState);
      profileState = profile.calculate(Constants.loopPeriodSecs);

      Logger.getInstance().recordOutput("Flywheels/SetpointRPM",
          profileState.position);
      Logger.getInstance().recordOutput("Flywheels/GoalRPM",
          profileGoal.position);
      Logger.getInstance().recordOutput("Flywheels/AtSetpoint", atSetpoint());
      Logger.getInstance().recordOutput("Flywheels/AtGoal", atGoal());

      double velocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(profileState.position);
      io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    }

    // Update LED mode
    leds.setFlywheelsReady(atGoal());
  }

  /** Run at the specified voltage with no other processing. Only use when characterizing. */
  public void runVoltage(double volts) {
    io.setVoltage(volts);
    closedLoop = false;
  }

  /** Run at velocity with closed loop control. */
  public void runVelocity(double rpm) {
    if (rpm > 0)
      rpm += offsetRpm;
    rpm = MathUtil.clamp(rpm, -maxVelocityRpm.get(), maxVelocityRpm.get());
    profileGoal = new TrapezoidProfile.State(rpm, 0.0);
    if (!closedLoop) {
      profileState = new TrapezoidProfile.State(getVelocity(), 0.0);
    }
    closedLoop = true;
  }

  /** Stops by going to open loop. */
  public void stop() {
    runVoltage(0.0);
  }

  /** Returns the current velocity of the flywheel in RPM. */
  public double getVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns whether the velocity has reached the closed loop setpoint. */
  public boolean atSetpoint() {
    if (closedLoop) {
      return Math.abs(getVelocity() - profileGoal.position) < toleranceRpm
          .get();
    } else {
      return false;
    }
  }

  /** Returns whether the velocity setpoint has reached the goal. */
  public boolean atGoal() {
    if (closedLoop) {
      return Math.abs(
          profileGoal.position - profileState.position) < toleranceRpm.get();
    } else {
      return false;
    }
  }

  /** Returns velocity of flywheel in radians per second. Only use for characterization. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  /** Returns the applied voltage of the flywheel. Only use for characterization. */
  public double getCharacterizationAppliedVolts() {
    return inputs.appliedVolts;
  }

  /** Increases the velocity offset of the shooter by 10 RPM */
  public void incrementOffset() {
    offsetRpm += 10;
  }

  /** Decreases the velocity offset of the shooter by 10 RPM */
  public void decrementOffset() {
    offsetRpm -= 10;
  }
}
