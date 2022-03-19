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
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.HopperEject;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngleProfile;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class ThreeCargoAutoCrossMidline extends SequentialCommandGroup {
  private static final Pose2d firstTurnPosition = new Pose2d(
      TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A).getTranslation(),
      Rotation2d.fromDegrees(90.0));

  private static final double ejectDuration = 0.75;
  private static final Pose2d cargoPosition =
      FieldConstants.cargoFOpposite.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-60.0)));
  private static final Pose2d ejectPosition =
      cargoPosition.transformBy(GeomUtil.transformFromTranslation(-1.0, 0.0));
  private static final Pose2d shootPosition =
      TwoCargoAuto.calcAimedPose(FieldConstants.referenceA);

  /**
   * Creates a new ThreeCargoAutoAlternative. Collects a second cargo from around tarmac A, then
   * uses an opponent cargo to collect one of our cargo from the opposite side of the side.
   */
  public ThreeCargoAutoCrossMidline(RobotState robotState, Drive drive,
      Vision vision, Flywheels flywheels, Hood hood, Tower tower, Kicker kicker,
      Intake intake, Leds leds) {
    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_A, robotState, drive,
            vision, flywheels, hood, tower, kicker, intake, leds),
        deadline(
            sequence(
                new TurnToAngleProfile(drive, robotState,
                    TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)
                        .getRotation(),
                    firstTurnPosition.getRotation()),
                new MotionProfileCommand(drive, robotState, 0.0,
                    List.of(firstTurnPosition, ejectPosition), 0.0, false)
                        .deadlineWith(
                            new RunIntake(true, intake, tower, kicker, leds)),
                new HopperEject(intake, tower, kicker)
                    .withTimeout(ejectDuration),
                new InstantCommand(intake::extend, intake),
                new MotionProfileCommand(drive, robotState, 0.0,
                    List.of(ejectPosition, cargoPosition), 0.0, false)
                        .deadlineWith(
                            new RunIntake(true, intake, tower, kicker, leds)),
                new MotionProfileCommand(drive, robotState, 0.0,
                    List.of(cargoPosition, shootPosition), 0.0, true),
                new Shoot(tower, kicker, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, tower,
                shootPosition.getTranslation())));
  }
}
