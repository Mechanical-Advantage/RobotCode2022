// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class RunClimberToPosition extends CommandBase {
  private final Climber climber;
  private final boolean top;

  /** Creates a new RunClimberToPosition. */
  public RunClimberToPosition(Climber climber, boolean top) {
    addRequirements(climber);
    this.climber = climber;
    this.top = top;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setGoal(top ? climber.positionLimitRad.get() : 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/RunClimberToPosition",
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.atGoal();
  }
}
