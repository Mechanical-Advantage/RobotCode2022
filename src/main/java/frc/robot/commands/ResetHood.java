// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.hood.Hood;

public class ResetHood extends SequentialCommandGroup {
  /** Creates a new ResetHood. Drives the hood to the bottom and resets it. */
  public ResetHood(Hood hood) {
    addCommands(
        new InstantCommand(() -> hood.moveToBottom(), hood),
        new WaitCommand(0.1),
        new WaitUntilCommand(hood::atGoal),
        new InstantCommand(() -> hood.reset(), hood));
  }
}
