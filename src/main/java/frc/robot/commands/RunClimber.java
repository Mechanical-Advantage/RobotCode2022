// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.TunableNumber;

public class RunClimber extends CommandBase {
  private static final TunableNumber maxVelocityRadPerSec =
      new TunableNumber("RunClimber/MaxVelocity");

  private final Climber climber;
  private final Supplier<Double> axisSupplier;
  private final Supplier<Boolean> openLoopSupplier;

  /** Creates a new RunClimber. */
  public RunClimber(Climber climber, Supplier<Double> axisSupplier,
      Supplier<Boolean> openLoopSupplier) {
    addRequirements(climber);
    this.climber = climber;
    this.axisSupplier = axisSupplier;
    this.openLoopSupplier = openLoopSupplier;
    maxVelocityRadPerSec.setDefault(35.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/RunClimber", true);

    double value = axisSupplier.get();
    double scaledValue = 0.0;
    if (Math.abs(value) > DriveWithJoysticks.getDeadband()) {
      scaledValue = (Math.abs(value) - DriveWithJoysticks.getDeadband())
          / (1 - DriveWithJoysticks.getDeadband());
      scaledValue = Math.copySign(scaledValue * scaledValue, value);
    }

    if (openLoopSupplier.get()) {
      climber.runPercent(scaledValue);
    } else {
      double newGoal = climber.getGoal() + (scaledValue
          * maxVelocityRadPerSec.get() * Constants.loopPeriodSecs);
      climber.setGoal(newGoal < 0.0 ? 0.0 : newGoal);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
