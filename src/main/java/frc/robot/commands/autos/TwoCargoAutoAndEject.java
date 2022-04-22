// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
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

public class TwoCargoAutoAndEject extends SequentialCommandGroup {
  private static final Pose2d firstCargoTurnPosition = new Pose2d(
      TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A).getTranslation(),
      Rotation2d.fromDegrees(-120.0));
  private static final Pose2d firstCargoPosition =
      FieldConstants.cargoC.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90.0)));
  private static final Pose2d secondCargoTurnPosition =
      FieldConstants.cargoC.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
  private static final Pose2d secondCargoPosition =
      FieldConstants.cargoA.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
  private static final Pose2d hangarEjectPosition =
      FieldConstants.referenceA.transformBy(new Transform2d(
          new Translation2d(2.0, 1.0), Rotation2d.fromDegrees(-150.0)));
  private static final Pose2d fenderEjectTurnPosition =
      FieldConstants.cargoA.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(135.0)));
  private static final Pose2d fenderEjectPosition =
      FieldConstants.fenderA.transformBy(new Transform2d(
          new Translation2d(0.75, 0.0), Rotation2d.fromDegrees(180.0)));

  /**
   * Creates a new TwoCargoAutoAndEject. Collects a second cargo from around tarmac A, then collects
   * the opponent cargo from tarmacs A & B to eject in the hangar or on the fender.
   */
  public TwoCargoAutoAndEject(boolean ejectHangar, RobotState robotState,
      Drive drive, Vision vision, Flywheels flywheels, Hood hood, Feeder feeder,
      Intake intake, Leds leds) {

    Command ejectCommand;
    if (ejectHangar) {
      ejectCommand = sequence(
          new MotionProfileCommand(drive, robotState, 0.0,
              List.of(secondCargoPosition, hangarEjectPosition), 0.0, true)
                  .deadlineWith(
                      new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
          new Shoot(feeder, leds).withTimeout(5.0))
              .deadlineWith(new PrepareShooterPreset(flywheels, hood, feeder,
                  ShooterPreset.HANGAR_EJECT));
    } else {
      ejectCommand = sequence(
          new TurnToAngleProfile(drive, robotState,
              secondCargoPosition.getRotation(),
              fenderEjectTurnPosition.getRotation()),
          new MotionProfileCommand(drive, robotState, 0.0,
              List.of(fenderEjectTurnPosition, fenderEjectPosition), 0.0, false)
                  .deadlineWith(
                      new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
          new RunIntake(IntakeMode.BACKWARDS, intake, feeder, leds)
              .withTimeout(5.0));
    }

    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_A, robotState, drive,
            vision, flywheels, hood, feeder, intake, leds),
        sequence(
            new TurnToAngleProfile(drive, robotState,
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)
                    .getRotation(),
                firstCargoTurnPosition.getRotation()),
            new MotionProfileCommand(drive, robotState, 0.0,
                List.of(firstCargoTurnPosition, firstCargoPosition), 0.0,
                false),
            new TurnToAngleProfile(drive, robotState,
                firstCargoPosition.getRotation(),
                secondCargoTurnPosition.getRotation()),
            new MotionProfileCommand(drive, robotState, 0.0,
                List.of(secondCargoTurnPosition, secondCargoPosition), 0.0,
                false)).deadlineWith(
                    new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
        ejectCommand);
  }
}
