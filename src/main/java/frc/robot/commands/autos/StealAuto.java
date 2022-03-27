// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.PrepareShooterPreset;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngleProfile;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;

public class StealAuto extends SequentialCommandGroup {
  private static final Pose2d opponentTurnPosition =
      TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A).transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
  private static final Pose2d opponentCargoPosition =
      FieldConstants.cargoA.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
  private static final Pose2d ejectPosition =
      FieldConstants.referenceA.transformBy(new Transform2d(
          new Translation2d(2.0, 1.0), Rotation2d.fromDegrees(-150.0)));

  /** Creates a new StealAuto. */
  public StealAuto(RobotState robotState, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Tower tower, Kicker kicker, Intake intake,
      Leds leds) {
    addCommands(
        deadline(
            sequence(
                new WaitUntilCommand(() -> flywheels.atGoal() && hood.atGoal()),
                new Shoot(tower, kicker, leds).withTimeout(0.5)),
            new PrepareShooterPreset(flywheels, hood, tower,
                ShooterPreset.UPPER_FENDER)),
        deadline(
            sequence(new InstantCommand(intake::extend, intake),
                deadline(
                    sequence(new WaitCommand(1.5),
                        new MotionProfileCommand(drive, robotState, 0.0,
                            List.of(AutoPosition.FENDER_A.getPose(),
                                TwoCargoAuto.cargoPositions
                                    .get(AutoPosition.TARMAC_A)),
                            0.0, false),
                        new WaitCommand(2.0)),
                    new RunIntake(true, intake, tower, kicker, leds)),
                new Shoot(tower, kicker, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, tower,
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)
                    .getTranslation())),
        deadline(
            sequence(
                sequence(
                    new TurnToAngleProfile(drive, robotState,
                        TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)
                            .getRotation(),
                        opponentTurnPosition.getRotation()),
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(opponentTurnPosition, opponentCargoPosition),
                        0.0, false),
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(opponentCargoPosition, ejectPosition), 0.0,
                        true)).deadlineWith(
                            new RunIntake(true, intake, tower, kicker, leds)),
                new Shoot(tower, kicker, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterPreset(flywheels, hood, tower,
                ShooterPreset.HANGAR_EJECT)));
  }
}
