// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngleProfile;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class FiveCargoAuto extends SequentialCommandGroup {
  public static final double firstShotLateSecs = 0.25; // How long after stop to begin feeding
  public static final double firstShotDurationSecs = 1.5; // How long to feed
  public static final double secondShotLateSecs = 0.0; // How long after stop to begin feeding
  public static final double secondShotDurationSecs = 0.6; // How long to feed
  public static final double thirdShotDurationSecs = 1.0; // How long to feed

  public static final double alertEarlySecs = 1.0; // How long before terminal arrival to light LEDs
  public static final double endTime = 14.9; // Finish routine at this time (includes some margin)

  public static final Pose2d terminalBackUpPosition =
      FourCargoAuto.terminalCargoPosition
          .transformBy(GeomUtil.transformFromTranslation(-0.75, 0.0));
  public static final TrajectoryConstraint terminalBackUpConstraint =
      new RectangularRegionConstraint(
          FourCargoAuto.terminalCargoPosition.getTranslation(),
          terminalBackUpPosition.getTranslation(),
          new MaxVelocityConstraint(Units.inchesToMeters(15.0)));
  public static final Pose2d thirdShotPosition =
      TwoCargoAuto.calcAimedPose(FourCargoAuto.terminalCargoPosition
          .transformBy(GeomUtil.transformFromTranslation(-2.5, 0.5)));
  public static final Pose2d thirdShotApproachPosition = thirdShotPosition
      .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));

  /**
   * Creates a new FiveCargoAuto. Collects the cargo surrounding tarmac CD and two cargo from the
   * terminal.
   */
  public FiveCargoAuto(RobotState robotState, Drive drive, Vision vision,
      Flywheels flywheels, Hood hood, Feeder feeder, Intake intake, Leds leds) {

    // Set up motion profiles
    MotionProfileCommand startToFirstCargo =
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(AutoPosition.TARMAC_D.getPose(),
                TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D)),
            0.0, false);
    TurnToAngleProfile firstShotToSecondCargoTurn =
        new TurnToAngleProfile(drive, robotState,
            TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D)
                .getRotation(),
            ThreeCargoAuto.tarmacDCTurnPosition.getRotation());
    MotionProfileCommand firstShotToSecondCargo = new MotionProfileCommand(
        drive, robotState, 0.0, List.of(ThreeCargoAuto.tarmacDCTurnPosition,
            ThreeCargoAuto.tarmacCCargoPosition),
        0.0, false);
    MotionProfileCommand secondCargoToSecondShot = new MotionProfileCommand(
        drive, robotState, 0.0, List.of(ThreeCargoAuto.tarmacCCargoPosition,
            ThreeCargoAuto.tarmacCShootPosition),
        0.0, true);
    MotionProfileCommand secondShotToTerminal =
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(ThreeCargoAuto.tarmacCShootPosition,
                FourCargoAuto.terminalCargoApproachPosition,
                FourCargoAuto.terminalCargoPosition),
            0.0, false);
    MotionProfileCommand terminalToThirdShot =
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(FourCargoAuto.terminalCargoPosition, terminalBackUpPosition,
                thirdShotApproachPosition, thirdShotPosition),
            0.0, true, List.of(terminalBackUpConstraint));

    // Full driving seqeuence, including waits
    double terminalArrivalTime =
        startToFirstCargo.getDuration() + firstShotLateSecs
            + firstShotDurationSecs + firstShotToSecondCargoTurn.getDuration()
            + firstShotToSecondCargo.getDuration()
            + secondCargoToSecondShot.getDuration() + secondShotLateSecs
            + secondShotDurationSecs + secondShotToTerminal.getDuration();
    double terminalLeaveTime =
        endTime - thirdShotDurationSecs - terminalToThirdShot.getDuration();
    double terminalWaitSecs = terminalLeaveTime - terminalArrivalTime;
    terminalWaitSecs = terminalWaitSecs < 0 ? 0 : terminalWaitSecs;
    Command driveSequence = sequence(startToFirstCargo,
        new WaitCommand(firstShotLateSecs + firstShotDurationSecs),
        firstShotToSecondCargoTurn, firstShotToSecondCargo,
        secondCargoToSecondShot,
        new WaitCommand(secondShotLateSecs + secondShotDurationSecs),
        secondShotToTerminal, new WaitCommand(terminalWaitSecs),
        terminalToThirdShot);

    // Shooting sequence, runs in parallel
    double firstShotStart = startToFirstCargo.getDuration() + firstShotLateSecs;
    double secondShotStart = startToFirstCargo.getDuration() + firstShotLateSecs
        + firstShotDurationSecs + firstShotToSecondCargoTurn.getDuration()
        + firstShotToSecondCargo.getDuration()
        + secondCargoToSecondShot.getDuration() + secondShotLateSecs;
    double thirdShotStart = endTime - thirdShotDurationSecs;
    Command shootSequence = sequence(
        new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds, true)
            .withTimeout(firstShotStart),
        new Shoot(feeder, leds)
            .alongWith(
                new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds, true)) // Continue rollers
            .withTimeout(firstShotDurationSecs),
        new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds).withTimeout(
            secondShotStart - firstShotStart - firstShotDurationSecs),
        new Shoot(feeder, leds).withTimeout(secondShotDurationSecs),
        new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds).withTimeout(
            thirdShotStart - secondShotStart - secondShotDurationSecs),
        new Shoot(feeder, leds).withTimeout(thirdShotDurationSecs));

    // Prepare shooter sequence, runs in parallel
    Command prepareShooterSequence = sequence(
        new PrepareShooterAuto(flywheels, hood, feeder,
            TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_D)
                .getTranslation())
                    .withTimeout(firstShotStart + firstShotDurationSecs),
        new PrepareShooterAuto(flywheels, hood, feeder,
            ThreeCargoAuto.tarmacCShootPosition.getTranslation())
                .withTimeout(secondShotStart + secondShotDurationSecs
                    - firstShotStart - firstShotDurationSecs),
        new PrepareShooterAuto(flywheels, hood, feeder,
            thirdShotPosition.getTranslation()));

    // LED sequence, runs in parallel
    double alertStart = terminalArrivalTime - alertEarlySecs;
    double alertDuration = terminalLeaveTime - alertStart;
    Command ledSequence = sequence(new WaitCommand(alertStart),
        new StartEndCommand(() -> leds.setAutoAlert(true),
            () -> leds.setAutoAlert(false)).withTimeout(alertDuration));

    // Combine all commands
    addCommands(new InstantCommand(intake::extend, intake),
        deadline(parallel(driveSequence, shootSequence), prepareShooterSequence,
            ledSequence),
        new PrintCommand(String.format(
            "*** Five cargo auto completed with %.2f sec terminal wait ***",
            terminalWaitSecs)));
  }
}
