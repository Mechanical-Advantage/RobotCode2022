// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class RunClimberToBottom extends CommandBase {
  private final Climber climber;
  private final double volts;

  /** Creates a new RunClimberToBottom. */
  public RunClimberToBottom(Climber climber, double volts) {
    addRequirements(climber);
    this.climber = climber;
    this.volts = volts < 0.0 ? 0.0 : volts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.runVoltage(-volts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/RunClimberToBottom",
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getLimitsActive();
  }
}
