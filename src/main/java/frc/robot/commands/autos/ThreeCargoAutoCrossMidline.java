// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimSimple;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.PrepareShooterPreset;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfile;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.CargoColor;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;

public class ThreeCargoAutoCrossMidline extends SequentialCommandGroup {
  private static final Pose2d firstTurnPosition = new Pose2d(
      TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A).getTranslation(),
      Rotation2d.fromDegrees(90.0));

  private static final double firstShootSecs = 0.6;
  private static final double ejectAlignDuration = 0.25;
  private static final double ejectDuration = 1.3;
  private static final Pose2d cargoPosition =
      FieldConstants.cargoFOpposite.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-60.0)));
  private static final Pose2d ejectPosition =
      cargoPosition.transformBy(new Transform2d(new Translation2d(-1.45, 0.0),
          Rotation2d.fromDegrees(0.0)));
  private static final Pose2d collectPositionRed =
      cargoPosition.transformBy(GeomUtil.transformFromTranslation(0.3, 0.0));
  private static final Pose2d collectPositionBlue =
      cargoPosition.transformBy(GeomUtil.transformFromTranslation(0.15, -0.2));
  private static final TrajectoryConstraint collectConstraint =
      new MaxVelocityConstraint(Units.inchesToMeters(120.0));
  private static final double cargoTrackingHoodAngle = 75.0;
  private static final double intakeWaitSecs = 0.2;
  private static final double autoAimTimeout = 1.2;
  private static final double towerPercent = 0.35;

  /**
   * Creates a new ThreeCargoAutoAlternative. Collects a second cargo from around tarmac A, then
   * uses an opponent cargo to collect one of our cargo from the opposite side of the side.
   */
  public ThreeCargoAutoCrossMidline(RobotState robotState, Drive drive,
      Vision vision, Flywheels flywheels, Hood hood, Feeder feeder,
      Intake intake, Leds leds) {

    AutoAimSimple cargoTrackingAutoAim = new AutoAimSimple(drive, vision, true);

    Supplier<Command> shootSequenceSupplier =
        () -> sequence(new WaitCommand(0.1),
            new WaitUntilCommand(() -> flywheels.atGoal() && hood.atGoal()),
            new Shoot(feeder, leds).raceWith(sequence(
                new WaitUntilCommand(() -> feeder.getUpperProxSensor()),
                new WaitUntilCommand(() -> !feeder.getUpperProxSensor()))),
            new WaitCommand(0.15));
    Supplier<Command> shootOwnCommandSupplier =
        () -> shootSequenceSupplier.get().deadlineWith(
            new PrepareShooterAuto(flywheels, hood, null, robotState));
    Supplier<Command> shootOpponentCommandSupplier =
        () -> shootSequenceSupplier.get().deadlineWith(new PrepareShooterPreset(
            flywheels, hood, null, ShooterPreset.OPPONENT_EJECT));

    MonitorColor monitorColorCommand = new MonitorColor(feeder);
    Command selectedShootCommand = new SelectCommand(
        Map.of(false,
            sequence(shootOwnCommandSupplier.get(),
                shootOpponentCommandSupplier.get()),
            true,
            sequence(shootOpponentCommandSupplier.get(),
                shootOwnCommandSupplier.get())),
        monitorColorCommand::opponentIsFirst);

    addCommands(
        new TwoCargoAuto(false, AutoPosition.TARMAC_A, robotState, drive,
            vision, flywheels, hood, feeder, intake, leds, firstShootSecs),
        new InstantCommand(() -> hood.moveToAngle(cargoTrackingHoodAngle),
            hood),
        new InstantCommand(() -> cargoTrackingAutoAim.updatePipeline(), hood),
        new TurnToAngleProfile(drive, robotState,
            TwoCargoAuto.cargoPositions.get(AutoPosition.TARMAC_A)
                .getRotation(),
            firstTurnPosition.getRotation()),
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(firstTurnPosition, ejectPosition), 0.0, false).deadlineWith(
                new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds)),
        new TurnToAngle(drive, robotState,
            ejectPosition.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)),
            false),
        cargoTrackingAutoAim.perpetually().withTimeout(ejectAlignDuration),
        new InstantCommand(() -> hood.moveToIdle(), hood),
        new TurnToAngle(drive, robotState, Rotation2d.fromDegrees(180.0), true),
        new RunIntake(IntakeMode.BACKWARDS_SLOW, intake, feeder, leds)
            .withTimeout(ejectDuration),
        deadline(
            sequence(
                new SelectCommand(
                    Map.of(Alliance.Red,
                        new MotionProfileCommand(drive, robotState, 0.0,
                            List.of(ejectPosition, collectPositionRed), 0.0,
                            false, List.of(collectConstraint)),
                        Alliance.Blue,
                        new MotionProfileCommand(drive, robotState, 0.0,
                            List.of(ejectPosition, collectPositionBlue), 0.0,
                            false, List.of(collectConstraint))),
                    DriverStation::getAlliance),
                new WaitCommand(intakeWaitSecs),
                new InstantCommand(intake::retract),
                new AutoAim(drive, robotState, vision, null, null).perpetually()
                    .withTimeout(autoAimTimeout)),
            new RunIntake(IntakeMode.FORWARDS, intake, feeder, leds),
            new PrepareShooterPreset(flywheels, hood, feeder,
                ShooterPreset.OPPONENT_EJECT),
            monitorColorCommand),
        new InstantCommand(() -> feeder.requestTowerShootPercent(towerPercent)),
        selectedShootCommand
            .deadlineWith(new StartEndCommand(() -> vision.setForceLeds(true),
                () -> vision.setForceLeds(false), vision)));
  }

  private static class MonitorColor extends CommandBase {
    private final Feeder feeder;
    private CargoColor color;

    public MonitorColor(Feeder feeder) {
      this.feeder = feeder;
    }

    @Override
    public void initialize() {
      color = null;
    }

    @Override
    public void execute() {
      if (color == null && feeder.getColorSensorProx()
          && feeder.getCargoColor() != CargoColor.UNKNOWN) {
        color = feeder.getCargoColor();
      }
    }

    public boolean opponentIsFirst() {
      switch (DriverStation.getAlliance()) {
        case Blue:
          return color == CargoColor.RED;
        case Red:
          return color == CargoColor.BLUE;
        default:
          return false;
      }
    }
  }
}
