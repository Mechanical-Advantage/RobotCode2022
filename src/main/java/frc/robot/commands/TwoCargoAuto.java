// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.PrepareShooter.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class TwoCargoAuto extends SequentialCommandGroup {
  public static final Map<AutoPosition, Pose2d> cargoPositions =
      Map.of(AutoPosition.TARMAC_A,
          FieldConstants.cargoB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)),
          AutoPosition.TARMAC_C,
          FieldConstants.cargoD.transformBy(new Transform2d(
              new Translation2d(0.1, 0.0), Rotation2d.fromDegrees(-45.0))),
          AutoPosition.TARMAC_D, FieldConstants.cargoE
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)));
  public static final Map<AutoPosition, Pose2d> shootPositions =
      Map.of(AutoPosition.TARMAC_A,
          calcAimedPose(AutoPosition.TARMAC_A.getPose()
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))),
          AutoPosition.TARMAC_C,
          calcAimedPose(AutoPosition.TARMAC_C.getPose()
              .transformBy(GeomUtil.transformFromTranslation(-0.4, 0.4))),
          AutoPosition.TARMAC_D, calcAimedPose(AutoPosition.TARMAC_D.getPose()
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))));

  /** Creates a new TwoCargoAuto. */
  public TwoCargoAuto(AutoPosition position, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Tower tower, Kicker kicker,
      Intake intake) {
    this(position.getPose(), OneCargoAuto.shootDurationSecs, position, drive,
        vision, flywheels, hood, tower, kicker, intake);
  }

  /** Creates a new TwoCargoAuto. */
  public TwoCargoAuto(Pose2d startingPose, double shootDurationSecs,
      AutoPosition position, Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake) {
    addCommands(deadline(
        sequence(new InstantCommand(intake::extend, intake),
            new WaitForVision(drive),
            sequence(
                new MotionProfileCommand(drive, 0.0,
                    List.of(startingPose, cargoPositions.get(position)), 0.0,
                    false),
                new MotionProfileCommand(drive, 0.0,
                    List.of(cargoPositions.get(position),
                        shootPositions.get(position)),
                    0.0, true)).deadlineWith(new RunIntake(intake, true)),
            new WaitUntilCommand(flywheels::atSetpoints),
            new Shoot(tower, kicker).withTimeout(shootDurationSecs)),
        new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER)));
  }

  public static Pose2d calcAimedPose(Pose2d pose) {
    Translation2d vehicleToCenter =
        FieldConstants.hubCenter.minus(pose.getTranslation());
    Rotation2d targetRotation =
        new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
    targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
    return new Pose2d(pose.getTranslation(), targetRotation);
  }
}
