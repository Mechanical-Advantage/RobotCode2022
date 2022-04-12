// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.hood.Hood;

public class ResetHood extends SequentialCommandGroup {
  /** Creates a new ResetHood. Drives the hood to the bottom and resets it. */
  public ResetHood(Hood hood) {
    addCommands(new InstantCommand(() -> hood.moveToBottom(), hood),
        new WaitCommand(0.1), new WaitUntilCommand(hood::atGoal),
        new InstantCommand(() -> hood.reset(), hood));
  }
}
