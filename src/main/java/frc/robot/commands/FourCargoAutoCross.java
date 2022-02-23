// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LedSelector;

public class FourCargoAutoCross extends SequentialCommandGroup {
  private static final double terminalWaitSecs = 1.5;

  /** Creates a new FourCargoAutoCross. */
  public FourCargoAutoCross(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake, LedSelector leds) {
    MotionProfileCommand firstShotToTerminal =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_A),
                FourCargoAuto.terminalCargoApproachPosition,
                FourCargoAuto.terminalCargoPosition),
            0.0, false);
    MotionProfileCommand terminalToSecondShot =
        new MotionProfileCommand(drive, 0.0,
            List.of(FourCargoAuto.terminalCargoPosition,
                TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_A)),
            0.0, true);

    addCommands(
        new TwoCargoAuto(AutoPosition.TARMAC_A, drive, vision, flywheels, hood,
            tower, kicker, intake, leds),
        deadline(
            sequence(
                sequence(firstShotToTerminal, new WaitCommand(terminalWaitSecs),
                    terminalToSecondShot).deadlineWith(
                        new RunIntake(true, intake, tower, kicker, leds)),
                new Shoot(tower, kicker, leds)
                    .withTimeout(OneCargoAuto.shootDurationSecs)),
            sequence(
                new WaitCommand(firstShotToTerminal.getDuration()
                    - FiveCargoAuto.alertEarlySecs),
                new StartEndCommand(() -> leds.setAutoAlert(true),
                    () -> leds.setAutoAlert(false)).withTimeout(
                        FiveCargoAuto.alertEarlySecs + terminalWaitSecs)),
            new PrepareShooterPreset(flywheels, hood,
                ShooterPreset.UPPER_FENDER)));
  }
}
