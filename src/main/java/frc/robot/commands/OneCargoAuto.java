// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.PrepareShooter.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCargoAuto extends SequentialCommandGroup {
  private static final double shootDurationSecs = 5.0;

  /** Creates a new OneCargoAuto. */
  public OneCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker) {
    addCommands(new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new AutoAim(drive, vision).alongWith(new WaitForVision(drive)),
            new WaitUntilCommand(flywheels::atSetpoints),
            new Shoot(tower, kicker).withTimeout(shootDurationSecs)),
        new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER)));
  }
}
