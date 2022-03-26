// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.Leds;


public class AutoClimb extends SequentialCommandGroup {
  private static double downPullVolts = 6.0;

  /** Creates a new AutoClimb. */
  public AutoClimb(Climber climber, Drive drive, Leds leds) {
    addCommands(new RunClimberToPosition(climber, climber.minPositionRad.get()),
        new RunClimberToBottom(climber, downPullVolts), new WaitCommand(0.9),
        new RunClimberToPosition(climber, climber.maxPositionRad.get()),
        new WaitCommand(0.6),
        new RunClimberToPosition(climber, climber.minPositionRad.get()),
        new RunClimberToBottom(climber, downPullVolts), new WaitCommand(0.6),
        new RunClimberToPosition(climber, climber.maxPositionRad.get()),
        new WaitCommand(0.5), new RunClimberToPosition(climber, 10.0),
        new InstantCommand(() -> leds.setClimbSuccess(true)));
  }
}
