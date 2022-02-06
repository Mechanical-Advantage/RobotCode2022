// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.PrepareShooter.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;

public class FiveCargoAuto extends SequentialCommandGroup {
  private static final double terminalWaitSecs = 0.5;
  private static final double shootDurationSecs = 1.0;
  private static final double secondShootStart = 0.5; // Time before reaching final position

  /** Creates a new FiveCargoAuto. */
  public FiveCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake) {

    MotionProfileCommand firstShootToCargoD =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_D),
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C)),
            0.0, false);
    MotionProfileCommand cargoDToSecondShoot =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C),
                TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_C)),
            0.0, true);

    addCommands(
        new TwoCargoAuto(AutoPosition.TARMAC_D.getPose(), shootDurationSecs,
            AutoPosition.TARMAC_D, drive, vision, flywheels, hood, tower,
            kicker, intake),
        deadline(
            sequence(
                sequence(firstShootToCargoD, cargoDToSecondShoot,
                    new MotionProfileCommand(drive, 0.0,
                        List.of(
                            TwoCargoAuto.shootPositions
                                .get(AutoPosition.TARMAC_C),
                            FourCargoAuto.terminalCargoApproachPosition,
                            FourCargoAuto.terminalCargoPosition),
                        0.0, false),
                    new WaitCommand(terminalWaitSecs),
                    new MotionProfileCommand(drive, 0.0,
                        List.of(FourCargoAuto.terminalCargoPosition,
                            TwoCargoAuto.shootPositions
                                .get(AutoPosition.TARMAC_C)),
                        0.0, true)).deadlineWith(
                            new RunIntake(intake, true),
                            sequence(
                                new WaitCommand(firstShootToCargoD.getDuration()
                                    + cargoDToSecondShoot.getDuration()
                                    - secondShootStart)
                                        .alongWith(new AutoIndex(tower)),
                                new Shoot(tower, kicker).withTimeout(
                                    shootDurationSecs),
                                new AutoIndex(tower))),
                new WaitUntilCommand(flywheels::atSetpoints),
                new Shoot(tower, kicker).withTimeout(shootDurationSecs)),
            new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER)));
  }
}
