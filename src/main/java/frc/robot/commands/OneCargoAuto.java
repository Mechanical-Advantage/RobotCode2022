// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.PrepareShooter.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;

public class OneCargoAuto extends SequentialCommandGroup {
  public static final double shootDurationSecs = 2.0;

  /** Creates a new OneCargoAuto. */
  public OneCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker) {
    addCommands(
        deadline(
            sequence(new WaitForVision(drive), new AutoAim(drive, vision),
                new WaitUntilCommand(flywheels::atSetpoints),
                new Shoot(tower, kicker).withTimeout(shootDurationSecs)),
            new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER)),
        new SimpleTaxi(drive));
  }
}
