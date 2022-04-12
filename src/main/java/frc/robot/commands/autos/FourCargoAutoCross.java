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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.vision.Vision;

public class FourCargoAutoCross extends SequentialCommandGroup {
  public static final double terminalWaitSecs = 1.5;

  public static final Pose2d intermediatePoint =
      FieldConstants.referenceB.transformBy(new Transform2d(
          new Translation2d(1.5, 0.0), Rotation2d.fromDegrees(60.0)));

  /**
   * Creates a new FourCargoAutoCross. Collects the cargo around tarmac A and two cargo from the
   * terminal.
   */
  public FourCargoAutoCross(RobotState robotState, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Feeder feeder, Intake intake, Leds leds) {
    MotionProfileCommand firstShotToTerminal =
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A),
                intermediatePoint, FourCargoAuto.terminalCargoApproachPosition,
                FourCargoAuto.terminalCargoPosition),
            0.0, false);
    MotionProfileCommand terminalToSecondShot =
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(FourCargoAuto.terminalCargoPosition, intermediatePoint,
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)),
            0.0, true);

    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_A, robotState, drive,
            vision, flywheels, hood, feeder, intake, leds),
        deadline(sequence(
            sequence(firstShotToTerminal, new WaitCommand(terminalWaitSecs),
                terminalToSecondShot).deadlineWith(
                    new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
            new Shoot(feeder, leds)
                .withTimeout(OneCargoAuto.shootDurationSecs)),
            sequence(
                new WaitCommand(firstShotToTerminal.getDuration()
                    - FiveCargoAuto.alertEarlySecs),
                new StartEndCommand(() -> leds.setAutoAlert(true),
                    () -> leds.setAutoAlert(false)).withTimeout(
                        FiveCargoAuto.alertEarlySecs + terminalWaitSecs)),
            new PrepareShooterAuto(flywheels, hood, feeder,
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)
                    .getTranslation())));
  }
}
