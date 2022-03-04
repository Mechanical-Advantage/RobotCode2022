// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.util.TunableNumber;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  public final TunableNumber minPositionRad =
      new TunableNumber("Climber/MinPosition");
  public final TunableNumber maxPositionRad =
      new TunableNumber("Climber/MaxPosition");
  private final TunableNumber kP = new TunableNumber("Climber/Kp");
  private final TunableNumber kI = new TunableNumber("Climber/Ki");
  private final TunableNumber kD = new TunableNumber("Climber/Kd");
  private final TunableNumber toleranceRad =
      new TunableNumber("Climber/Tolerance");
  private final TunableNumber maxVelocity =
      new TunableNumber("Climber/MaxVelocity");
  private final TunableNumber maxAcceleration =
      new TunableNumber("Climber/MaxAcceleration");

  private final ProfiledPIDController controller = new ProfiledPIDController(
      0.0, 0.0, 0.0, new Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double basePositionRad = 0.0;
  private boolean closedLoop = false;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        minPositionRad.setDefault(-1.5);
        maxPositionRad.setDefault(36.0);
        kP.setDefault(2.5);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRad.setDefault(1.0);
        maxVelocity.setDefault(40.0);
        maxAcceleration.setDefault(120.0);
        break;
      default:
        minPositionRad.setDefault(0.0);
        maxPositionRad.setDefault(0.0);
        kP.setDefault(0.0);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceRad.setDefault(0.0);
        maxVelocity.setDefault(0.0);
        maxAcceleration.setDefault(0.0);
        break;
    }

    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Climber", inputs);

    // Unlock climber
    if (DriverStation.isEnabled()) {
      io.setUnlocked(true);
    }

    // Log position
    Logger.getInstance().recordOutput("Climber/Position", getPosition());
    Logger.getInstance().recordOutput("Climber/PositionSetpoint",
        controller.getSetpoint().position);

    // Update PID controller with tunable numbers
    if (kP.hasChanged()) {
      controller.setP(kP.get());
    }
    if (kI.hasChanged()) {
      controller.setI(kI.get());
    }
    if (kD.hasChanged()) {
      controller.setI(kD.get());
    }
    if (toleranceRad.hasChanged()) {
      controller.setTolerance(toleranceRad.get());
    }
    if (maxVelocity.hasChanged() || maxAcceleration.hasChanged()) {
      controller.setConstraints(
          new Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    // Run PID controller
    if (closedLoop) {
      double volts = controller.calculate(getPosition());
      io.setVoltage(volts);
    }
  }

  /**
   * Runs open loop when unlocked, disables PID control
   * 
   * @param percent Percent of full voltage
   */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
    closedLoop = false;
  }

  /**
   * Runs closed loop when unlocked to the specified position
   * 
   * @param positionRad Position in radians (between 0 and max height)
   */
  public void setGoal(double positionRad) {
    if (!closedLoop) {
      controller.reset(getPosition());
    }
    controller.setGoal(MathUtil.clamp(positionRad, minPositionRad.get(),
        maxPositionRad.get()));
    closedLoop = true;
  }

  /** Returns whether the climber is at the current goal */
  public boolean atGoal() {
    if (closedLoop) {
      return Math.abs(controller.getGoal().position
          - controller.getSetpoint().position) < 0.1;
    } else {
      return false;
    }
  }

  /** Returns the position goal in radians */
  public double getGoal() {
    if (closedLoop) {
      return controller.getGoal().position;
    } else {
      return getPosition();
    }
  }

  /** Returns the current position in radians */
  public double getPosition() {
    return inputs.positionRad - basePositionRad;
  }

  /** Returns the current velocity in radians per second */
  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  /** Returns the current draw in amps. */
  public double getCurrentAmps() {
    if (inputs.currentAmps.length > 0) {
      return inputs.currentAmps[0];
    } else {
      return 0.0;
    }
  }

  /** Resets the current position to zero */
  public void resetPosition() {
    basePositionRad = inputs.positionRad;
  }
}
