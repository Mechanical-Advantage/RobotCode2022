// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.climber.Climber;

public class ResetClimber extends SequentialCommandGroup {
  private static final double initialRaiseSecs = 0.5;
  private static final double initialRaiseVolts = 0.8;
  private static final double pullDownVolts = 0.5;

  /**
   * Creates a new ResetClimber.
   */
  public ResetClimber(Climber climber) {
    addCommands(
        new StartEndCommand(() -> climber.runVoltage(initialRaiseVolts),
            () -> climber.runVoltage(0.0), climber)
                .withTimeout(initialRaiseSecs),
        new RunClimberToBottom(climber, pullDownVolts),
        new InstantCommand(climber::resetPosition, climber));
  }
}
