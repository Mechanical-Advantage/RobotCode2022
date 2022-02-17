// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;

public class HPPractice extends SequentialCommandGroup {
  /** Creates a new HPPractice. */
  public HPPractice(Drive drive, Intake intake, Tower tower, Kicker kicker) {
    addCommands(new InstantCommand(intake::extend, intake),
        deadline(
            sequence(
                new MotionProfileCommand(drive, 0.0,
                    List.of(new Pose2d(),
                        new Pose2d(new Translation2d(5.0, 0.0),
                            new Rotation2d())),
                    0.0, false),
                new WaitCommand(FiveCargoAuto.terminalWaitSecs),
                new MotionProfileCommand(drive, 0.0,
                    List.of(new Pose2d(new Translation2d(5.0, 0.0),
                        new Rotation2d()), new Pose2d()),
                    0.0, true)),
            new RunIntake(true, intake, tower, kicker)));
  }
}
