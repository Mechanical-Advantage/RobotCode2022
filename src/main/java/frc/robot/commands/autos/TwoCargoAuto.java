// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class TwoCargoAuto extends SequentialCommandGroup {
  public static final double intakeLengthSecs = 1.0;

  public static final Map<AutoPosition, Pose2d> cargoPositions =
      Map.of(AutoPosition.TARMAC_A,
          calcAimedPose(FieldConstants.cargoB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))),
          AutoPosition.TARMAC_C,
          calcAimedPose(FieldConstants.cargoD
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))),
          AutoPosition.TARMAC_D, calcAimedPose(FieldConstants.cargoE
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0))));

  /** Creates a new TwoCargoAuto. Collects a second cargo from around the tarmac and shoots. */
  public TwoCargoAuto(boolean taxiFinish, AutoPosition position,
      RobotState robotState, Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake, Leds leds) {
    addCommands(
        deadline(
            sequence(new InstantCommand(intake::extend, intake),
                sequence(
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(position.getPose(),
                            cargoPositions.get(position)),
                        0.0, false),
                    new WaitCommand(intakeLengthSecs)).deadlineWith(
                        new RunIntake(true, intake, tower, kicker, leds)),
                new Shoot(tower, kicker, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, tower,
                cargoPositions.get(position).getTranslation())),
        taxiFinish ? new Taxi(drive, false) : new InstantCommand());
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
