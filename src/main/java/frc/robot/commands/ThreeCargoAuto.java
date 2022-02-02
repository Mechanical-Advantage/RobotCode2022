// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;

public class ThreeCargoAuto extends SequentialCommandGroup {
  /** Creates a new ThreeCargoAuto. */
  public ThreeCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake) {
    addCommands(
        new TwoCargoAuto(AutoPosition.TARMAC_D, drive, vision, flywheels, hood,
            tower, kicker, intake),
        new TwoCargoAuto(TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_D),
            AutoPosition.TARMAC_C, drive, vision, flywheels, hood, tower,
            kicker, intake));
  }
}
