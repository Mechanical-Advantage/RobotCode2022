// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.RobotState;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngleProfile;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;
import java.util.List;

public class FourCargoAuto extends SequentialCommandGroup {
  public static final Pose2d terminalCargoPosition =
      FieldConstants.cargoG.transformBy(
          new Transform2d(new Translation2d(0.35, 0.0), Rotation2d.fromDegrees(180.0)));
  public static final Pose2d terminalCargoApproachPosition =
      terminalCargoPosition.transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0));

  /** Creates a new FourCargoAuto. Collects the cargo surrounding tarmac CD and the terminal. */
  public FourCargoAuto(
      RobotState robotState,
      Drive drive,
      Vision vision,
      Flywheels flywheels,
      Hood hood,
      Feeder feeder,
      Intake intake,
      Leds leds) {
    addCommands(
        new TwoCargoAuto(
            false,
            AutoPosition.TARMAC_D,
            robotState,
            drive,
            vision,
            flywheels,
            hood,
            feeder,
            intake,
            leds),
        deadline(
            sequence(
                sequence(
                        new TurnToAngleProfile(
                            drive,
                            robotState,
                            TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D).getRotation(),
                            ThreeCargoAuto.tarmacDCTurnPosition.getRotation()),
                        new MotionProfileCommand(
                            drive,
                            robotState,
                            0.0,
                            List.of(
                                ThreeCargoAuto.tarmacDCTurnPosition,
                                FieldConstants.cargoD.transformBy(
                                    new Transform2d(
                                        new Translation2d(), Rotation2d.fromDegrees(-60.0))),
                                terminalCargoApproachPosition,
                                terminalCargoPosition),
                            0.0,
                            false),
                        new MotionProfileCommand(
                            drive,
                            robotState,
                            0.0,
                            List.of(
                                terminalCargoPosition,
                                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C)),
                            0.0,
                            true))
                    .deadlineWith(new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
                new Shoot(feeder, leds).withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(
                flywheels,
                hood,
                feeder,
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C).getTranslation())));
  }
}
