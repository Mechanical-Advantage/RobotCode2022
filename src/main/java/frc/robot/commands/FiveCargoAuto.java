// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private static final double firstShotStationarySecs = 1.0; // How long to stay still
  private static final double firstShotDurationSecs = 1.0; // How long to feed
  private static final double firstShotEarlySecs = 0.0; // How long before stop to begin feeding

  private static final double secondShotStationarySecs = 0.0; // How long to stay still
  private static final double secondShotDurationSecs = 1.0; // How long to feed
  private static final double secondShotEarlySecs = 0.5; // How long before stop to begin feeding

  private static final double thirdShotDurationSecs = 2.0; // How long to feed
  private static final double thirdShotEarlySecs = 1.0; // How long before stop to begin feeding

  private static final double terminalWaitSecs = 0.5;

  /** Creates a new FiveCargoAuto. */
  public FiveCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake) {

    // Set up motion profiles
    MotionProfileCommand startToFirstCargo =
        new MotionProfileCommand(drive, 0.0,
            List.of(AutoPosition.TARMAC_D.getPose(),
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D)),
            0.0, false);
    MotionProfileCommand firstCargoToFirstShot =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D),
                TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_D)),
            0.0, true);
    MotionProfileCommand firstShotToSecondCargo =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_D),
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C)),
            0.0, false);
    MotionProfileCommand secondCargoToSecondShot =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_C),
                TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_C)),
            0.0, true);
    MotionProfileCommand secondShotToTerminal =
        new MotionProfileCommand(drive, 0.0,
            List.of(TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_C),
                FourCargoAuto.terminalCargoApproachPosition,
                FourCargoAuto.terminalCargoPosition),
            0.0, false);
    MotionProfileCommand terminalToThirdShot =
        new MotionProfileCommand(drive, 0.0,
            List.of(FourCargoAuto.terminalCargoPosition,
                TwoCargoAuto.shootPositions.get(AutoPosition.TARMAC_C)),
            0.0, true);

    // Full driving seqeuence, including waits
    Command driveSequence = sequence(startToFirstCargo, firstCargoToFirstShot,
        new WaitCommand(firstShotStationarySecs), firstShotToSecondCargo,
        secondCargoToSecondShot, new WaitCommand(secondShotStationarySecs),
        secondShotToTerminal, terminalToThirdShot);

    // Shooting sequence, runs in parallel
    double firstShotStart = startToFirstCargo.getDuration()
        + firstCargoToFirstShot.getDuration() - firstShotEarlySecs;
    double secondShotStart =
        startToFirstCargo.getDuration() + firstCargoToFirstShot.getDuration()
            + firstShotStationarySecs + firstShotToSecondCargo.getDuration()
            + secondCargoToSecondShot.getDuration() - secondShotEarlySecs;
    double thirdShotStart =
        startToFirstCargo.getDuration() + firstCargoToFirstShot.getDuration()
            + firstShotStationarySecs + firstShotToSecondCargo.getDuration()
            + secondCargoToSecondShot.getDuration() + secondShotStationarySecs
            + secondShotToTerminal.getDuration() + terminalWaitSecs
            + terminalToThirdShot.getDuration() - thirdShotEarlySecs;
    Command shootSequence = sequence(
        new RunIntake(true, intake, tower, kicker).withTimeout(firstShotStart),
        new Shoot(tower, kicker).withTimeout(firstShotDurationSecs),
        new RunIntake(true, intake, tower, kicker).withTimeout(
            secondShotStart - firstShotStart - firstShotDurationSecs),
        new Shoot(tower, kicker).withTimeout(secondShotDurationSecs),
        new RunIntake(true, intake, tower, kicker).withTimeout(
            thirdShotStart - secondShotStart - secondShotDurationSecs),
        new Shoot(tower, kicker).withTimeout(thirdShotDurationSecs));

    // Combine all commands
    addCommands(new InstantCommand(intake::extend, intake),
        new WaitForVision(drive),
        deadline(parallel(driveSequence, shootSequence), new PrepareShooter(
            drive, flywheels, hood, ShooterPreset.UPPER_FENDER)));
  }
}
