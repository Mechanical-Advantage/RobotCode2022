// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LedSelector;

public class OneCargoAuto extends SequentialCommandGroup {
  public static final double shootDurationSecs = 1.5;

  /** Creates a new OneCargoAuto. */
  public OneCargoAuto(boolean longTaxi, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Tower tower, Kicker kicker,
      LedSelector leds) {
    addCommands(
        deadline(
            sequence(new WaitForVision(drive), new AutoAim(drive, vision),
                new WaitUntilCommand(flywheels::atSetpoint),
                new Shoot(tower, kicker, hood, leds)
                    .withTimeout(shootDurationSecs)),
            new PrepareShooterPreset(flywheels, hood,
                ShooterPreset.UPPER_FENDER)),
        new Taxi(drive, longTaxi));
  }
}
