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

public class FourCargoAutoAvoidD extends SequentialCommandGroup {
  public static final double terminalWaitSecs = 1.5;

  public static final Pose2d terminalCargoPosition =
      FieldConstants.cargoG.transformBy(new Transform2d(
          new Translation2d(0.2, 0.0), Rotation2d.fromDegrees(180.0)));
  public static final Pose2d terminalCargoApproachPosition =
      terminalCargoPosition
          .transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0));

  /**
   * Creates a new FourCargoAutoLimited. Collects the cargo around tarmac C and two from the
   * terminal.
   */
  public FourCargoAutoAvoidD(RobotState robotState, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Tower tower, Kicker kicker, Intake intake,
      Leds leds) {
    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_C, robotState, drive,
            vision, flywheels, hood, tower, kicker, intake, leds),
        deadline(
            sequence(
                sequence(
                    new MotionProfileCommand(drive, robotState, 0.0, List.of(
                        TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C),
                        terminalCargoApproachPosition, terminalCargoPosition),
                        0.0, false),
                    new WaitCommand(terminalWaitSecs),
                    new MotionProfileCommand(drive, robotState, 0.0,
                        List.of(terminalCargoPosition,
                            TwoCargoAuto.cargoPositions
                                .get(AutoPosition.TARMAC_C)),
                        0.0, true)).deadlineWith(
                            new RunIntake(true, intake, tower, kicker, leds)),
                new Shoot(tower, kicker, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, tower,
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C)
                    .getTranslation())));
  }
}
