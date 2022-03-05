// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class FourCargoAuto extends SequentialCommandGroup {
  public static final Pose2d terminalCargoPosition =
      FieldConstants.cargoG.transformBy(new Transform2d(
          new Translation2d(0.55, 0.0), Rotation2d.fromDegrees(180.0)));
  public static final Pose2d terminalCargoApproachPosition =
      terminalCargoPosition
          .transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0));

  /** Creates a new FourCargoAuto. */
  public FourCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake, Leds leds) {
    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_D, drive, vision, flywheels,
            hood, tower, kicker, intake, leds),
        deadline(
            sequence(
                sequence(new TurnToAngleProfile(drive,
                    TwoCargoAuto.cargoPositions.get(
                        AutoPosition.TARMAC_D).getRotation(),
                    ThreeCargoAuto.tarmacDCTurnPosition.getRotation()),
                    new MotionProfileCommand(drive, 0.0,
                        List.of(ThreeCargoAuto.tarmacDCTurnPosition,
                            FieldConstants.cargoD.transformBy(
                                new Transform2d(new Translation2d(),
                                    Rotation2d.fromDegrees(-60.0))),
                            terminalCargoApproachPosition,
                            terminalCargoPosition),
                        0.0, false),
                    new MotionProfileCommand(drive, 0.0,
                        List.of(terminalCargoPosition,
                            TwoCargoAuto.cargoPositions
                                .get(AutoPosition.TARMAC_C)),
                        0.0, true)).deadlineWith(
                            new RunIntake(true, intake, tower, kicker, leds)),
                new Shoot(tower, kicker, hood, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterPreset(flywheels, hood, tower,
                ShooterPreset.UPPER_TARMAC_HIGH)));
  }
}
