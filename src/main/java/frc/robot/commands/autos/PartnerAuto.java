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
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;

public class PartnerAuto extends SequentialCommandGroup {
  private static final Pose2d opponentTurnPosition =
      TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A).transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
  private static final Pose2d opponentCargoPosition =
      FieldConstants.cargoA.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
  private static final Pose2d ejectPosition =
      FieldConstants.referenceA.transformBy(new Transform2d(
          new Translation2d(2.0, 1.0), Rotation2d.fromDegrees(-150.0)));

  /**
   * Creates a new StealAuto. Collects a cargo from a partner robot, then ejects an opponent cargo
   * into the hangar.
   */
  public PartnerAuto(RobotState robotState, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Feeder feeder, Intake intake, Leds leds) {
    addCommands(
        deadline(
            sequence(
                new WaitUntilCommand(() -> flywheels.atGoal() && hood.atGoal()),
                new Shoot(feeder, leds).withTimeout(0.5)),
            new PrepareShooterPreset(flywheels, hood, feeder,
                ShooterPreset.UPPER_FENDER)),
        deadline(
            sequence(new InstantCommand(intake::extend, intake),
                sequence(new WaitCommand(1.5),
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(AutoPosition.FENDER_A.getPose(),
                            TwoCargoAuto.cargoPositions
                                .get(AutoPosition.TARMAC_A)),
                        0.0, false),
                    new WaitCommand(2.0))
                        .deadlineWith(new RunIntake(IntakeMode.FORWARDS, intake,
                            feeder, leds)),
                new Shoot(feeder, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, feeder,
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
                        true))
                            .deadlineWith(new RunIntake(IntakeMode.FORWARDS,
                                intake, feeder, leds)),
                new Shoot(feeder, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterPreset(flywheels, hood, feeder,
                ShooterPreset.HANGAR_EJECT)));
  }
}
