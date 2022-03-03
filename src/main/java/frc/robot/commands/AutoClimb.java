// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;


public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb(Climber climber, Drive drive) {
    addCommands(new RunClimberToPosition(climber, climber.minPositionRad.get()),
        new WaitCommand(1.0),
        new RunClimberToPosition(climber, climber.maxPositionRad.get()),
        new WaitCommand(0.5),
        new RunClimberToPosition(climber, climber.minPositionRad.get()),
        new WaitCommand(2.0),
        new RunClimberToPosition(climber, climber.maxPositionRad.get()),
        new WaitCommand(0.5), new RunClimberToPosition(climber, 20.0));
  }
}