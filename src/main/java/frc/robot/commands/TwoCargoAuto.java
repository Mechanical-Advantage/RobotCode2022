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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCargoAuto extends SequentialCommandGroup {
  private static final double shootDurationSecs = 5.0;

  public static final Map<AutoPosition, Pose2d> cargoPositions = Map.of(
      AutoPosition.TARMAC_A,
      FieldConstants.cargoB
          .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)),
      AutoPosition.TARMAC_D,
      FieldConstants.cargoE
          .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)),
      AutoPosition.TARMAC_C, FieldConstants.cargoD.transformBy(new Transform2d(
          new Translation2d(-0.4, 0.4), Rotation2d.fromDegrees(-45.0))));
  public static final Map<AutoPosition, Pose2d> shootPositions =
      Map.of(AutoPosition.TARMAC_A,
          FieldConstants.cargoB
              .transformBy(GeomUtil.transformFromTranslation(-1.5, 0.0)),
          AutoPosition.TARMAC_D,
          FieldConstants.cargoE
              .transformBy(GeomUtil.transformFromTranslation(-1.5, 0.0)),
          AutoPosition.TARMAC_C, FieldConstants.cargoD
              .transformBy(GeomUtil.transformFromTranslation(-1.5, 0.4)));

  /** Creates a new TwoCargoAuto. */
  public TwoCargoAuto(AutoPosition position, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Tower tower, Kicker kicker,
      Intake intake) {
    this(shootPositions.get(position), position, drive, vision, flywheels, hood,
        tower, kicker, intake);
  }

  /** Creates a new TwoCargoAuto. */
  public TwoCargoAuto(Pose2d startingPose, AutoPosition position, Drive drive,
      Vision vision, Flywheels flywheels, Hood hood, Tower tower, Kicker kicker,
      Intake intake) {
    addCommands(new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.extend(), intake),
            new WaitForVision(drive),
            new MotionProfileCommand(drive, 0.0,
                List.of(startingPose, cargoPositions.get(position)), 0.0, false)
                    .deadlineWith(new RunIntake(intake, true)),
            new MotionProfileCommand(drive, 0.0,
                List.of(cargoPositions.get(position),
                    shootPositions.get(position)),
                0.0, true),
            new AutoAim(drive, vision),
            new WaitUntilCommand(flywheels::atSetpoints),
            new Shoot(tower, kicker).withTimeout(shootDurationSecs)),
        new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER)));
  }
}
