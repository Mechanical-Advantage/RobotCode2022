// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngleProfile;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class ThreeCargoAuto extends SequentialCommandGroup {
  public static Pose2d tarmacDCTurnPosition =
      TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D).transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-120.0)));
  public static Pose2d tarmacCCargoPosition =
      FieldConstants.cargoD.transformBy(new Transform2d(
          new Translation2d(-0.2, 0.2), Rotation2d.fromDegrees(-45.0)));
  public static Pose2d tarmacCShootPosition =
      TwoCargoAuto.calcAimedPose(tarmacCCargoPosition
          .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1)));

  /** Creates a new ThreeCargoAuto. Collects the cargo surrounding tarmac CD and shoots. */
  public ThreeCargoAuto(RobotState robotState, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Feeder feeder, Intake intake, Leds leds) {
    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_D, robotState, drive,
            vision, flywheels, hood, feeder, intake, leds),
        deadline(
            sequence(
                sequence(
                    new TurnToAngleProfile(drive, robotState,
                        TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D)
                            .getRotation(),
                        tarmacDCTurnPosition.getRotation()),
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(tarmacDCTurnPosition, tarmacCCargoPosition),
                        0.0, false),
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(tarmacCCargoPosition, tarmacCShootPosition),
                        0.0, true))
                            .deadlineWith(new RunIntake(IntakeMode.FORWARDS,
                                intake, feeder, leds)),
                new Shoot(feeder, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, feeder,
                tarmacCShootPosition.getTranslation())),
        new Taxi(drive, false));
  }
}
