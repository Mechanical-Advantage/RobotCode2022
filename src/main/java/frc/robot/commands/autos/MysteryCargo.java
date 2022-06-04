// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.util.GeomUtil;

public class MysteryCargo extends SequentialCommandGroup {
  public static final Pose2d mysteryCollectPosition =
      FieldConstants.cargoMystery
          .transformBy(GeomUtil.transformFromTranslation(-0.2, 0.0));
  public static final Map<ShooterPreset, Rotation2d> shootRotations =
      Map.of(ShooterPreset.MYSTERY_STANDS_NEAR, Rotation2d.fromDegrees(-90.0),
          ShooterPreset.MYSTERY_STANDS_FAR, Rotation2d.fromDegrees(65.0),
          ShooterPreset.MYSTERY_ALLIANCE_STATION, new Rotation2d(),
          ShooterPreset.MYSTERY_HUMAN_PLAYER,
          GeomUtil.direction(mysteryCollectPosition.getTranslation()));

  /**
   * Creates a new MysteryCargo. Scores two cargo from tarmac A, then ejects the mystery cargo to
   * the stands, alliance station, or human player station.
   */
  public MysteryCargo(ShooterPreset mode, RobotState robotState, Drive drive,
      Vision vision, Flywheels flywheels, Hood hood, Feeder feeder,
      Intake intake, Leds leds) {
    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_A, robotState, drive,
            vision, flywheels, hood, feeder, intake, leds),
        deadline(
            sequence(new MotionProfileCommand(drive, robotState, 0.0,
                List.of(TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A),
                    mysteryCollectPosition),
                0.0, false).deadlineWith(
                    new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
                new TurnToAngleProfile(drive, robotState,
                    mysteryCollectPosition.getRotation(),
                    shootRotations.get(mode)),
                new Shoot(feeder, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterPreset(flywheels, hood, feeder, mode)));
  }
}
