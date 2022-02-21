// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

public class FiveCargoAuto extends SequentialCommandGroup {
  public static final double firstShotStationarySecs = 0.0; // How long to stay still
  public static final double firstShotDurationSecs = 1.0; // How long to feed
  public static final double firstShotEarlySecs = 0.5; // How long before stop to begin feeding

  public static final double secondShotStationarySecs = 0.0; // How long to stay still
  public static final double secondShotDurationSecs = 1.0; // How long to feed
  public static final double secondShotEarlySecs = 0.5; // How long before stop to begin feeding

  public static final double thirdShotDurationSecs = 1.0; // How long to feed
  public static final double thirdShotEarlySecs = 0.5; // How long before stop to begin feeding

  public static final double alertEarlySecs = 0.5; // How long before terminal arrival to light LEDs
  public static final double endTime = 14.9; // Finish routine at this time (includes some margin)

  /** Creates a new FiveCargoAuto. */
  public FiveCargoAuto(Drive drive, Vision vision, Flywheels flywheels,
      Hood hood, Tower tower, Kicker kicker, Intake intake, LedSelector leds) {

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
    double terminalArrivalTime =
        startToFirstCargo.getDuration() + firstCargoToFirstShot.getDuration()
            + firstShotStationarySecs + firstShotToSecondCargo.getDuration()
            + secondCargoToSecondShot.getDuration() + secondShotStationarySecs
            + secondShotToTerminal.getDuration();
    double terminalLeaveTime = endTime - thirdShotDurationSecs
        + thirdShotEarlySecs - terminalToThirdShot.getDuration();
    double terminalWaitSecs = terminalLeaveTime - terminalArrivalTime;
    terminalWaitSecs = terminalWaitSecs < 0 ? 0 : terminalWaitSecs;
    Command driveSequence = sequence(startToFirstCargo, firstCargoToFirstShot,
        new WaitCommand(firstShotStationarySecs), firstShotToSecondCargo,
        secondCargoToSecondShot, new WaitCommand(secondShotStationarySecs),
        secondShotToTerminal, new WaitCommand(terminalWaitSecs),
        terminalToThirdShot);

    // Shooting sequence, runs in parallel
    double firstShotStart = startToFirstCargo.getDuration()
        + firstCargoToFirstShot.getDuration() - firstShotEarlySecs;
    double secondShotStart =
        startToFirstCargo.getDuration() + firstCargoToFirstShot.getDuration()
            + firstShotStationarySecs + firstShotToSecondCargo.getDuration()
            + secondCargoToSecondShot.getDuration() - secondShotEarlySecs;
    double thirdShotStart = endTime - thirdShotDurationSecs;
    Command shootSequence = sequence(
        new RunIntake(true, intake, tower, kicker).withTimeout(firstShotStart),
        new Shoot(tower, kicker, leds).withTimeout(firstShotDurationSecs),
        new RunIntake(true, intake, tower, kicker).withTimeout(
            secondShotStart - firstShotStart - firstShotDurationSecs),
        new Shoot(tower, kicker, leds).withTimeout(secondShotDurationSecs),
        new RunIntake(true, intake, tower, kicker).withTimeout(
            thirdShotStart - secondShotStart - secondShotDurationSecs),
        new Shoot(tower, kicker, leds).withTimeout(thirdShotDurationSecs));

    // LED sequence, runs in parallel
    double alertStart = terminalArrivalTime - alertEarlySecs;
    double alertDuration = terminalLeaveTime - alertStart;
    Command ledSequence = sequence(new WaitCommand(alertStart),
        new StartEndCommand(() -> leds.setAutoAlert(true),
            () -> leds.setAutoAlert(false)).withTimeout(alertDuration));

    // Combine all commands
    addCommands(new InstantCommand(intake::extend, intake),
        deadline(parallel(driveSequence, shootSequence, ledSequence),
            new PrepareShooterPreset(flywheels, hood,
                ShooterPreset.UPPER_FENDER)),
        new PrintCommand(String.format(
            "*** Five cargo auto completed with %.2f sec terminal wait ***",
            terminalWaitSecs)));
  }
}
