// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import java.util.List;

public class HPPractice extends SequentialCommandGroup {
  private static final double terminalWaitSecs = 1.5;

  /** Creates a new HPPractice. */
  public HPPractice(RobotState robotState, Drive drive, Intake intake, Feeder feeder, Leds leds) {
    MotionProfileCommand driveForwards =
        new MotionProfileCommand(
            drive,
            robotState,
            0.0,
            List.of(new Pose2d(), new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d())),
            0.0,
            false);
    MotionProfileCommand driveBackwards =
        new MotionProfileCommand(
            drive,
            robotState,
            0.0,
            List.of(new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d()), new Pose2d()),
            0.0,
            true);

    addCommands(
        new InstantCommand(intake::extend, intake),
        deadline(
            sequence(driveForwards, new WaitCommand(terminalWaitSecs), driveBackwards),
            sequence(
                new WaitCommand(driveForwards.getDuration() - FiveCargoAuto.alertEarlySecs),
                new StartEndCommand(() -> leds.setAutoAlert(true), () -> leds.setAutoAlert(false))
                    .withTimeout(FiveCargoAuto.alertEarlySecs + terminalWaitSecs)),
            new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)));
  }
}
