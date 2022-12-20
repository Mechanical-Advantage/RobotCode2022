// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import org.littletonrobotics.junction.Logger;

public class RunClimberToPosition extends CommandBase {
  private final Climber climber;
  private final double position;

  /** Creates a new RunClimberToPosition. */
  public RunClimberToPosition(Climber climber, boolean top) {
    this(climber, top ? climber.maxPositionRad.get() : climber.minPositionRad.get());
  }

  /** Creates a new RunClimberToPosition. */
  public RunClimberToPosition(Climber climber, double position) {
    addRequirements(climber);
    this.climber = climber;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setGoal(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/RunClimberToPosition", true);
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
