// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoIndex;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FiveCargoAuto;
import frc.robot.commands.FourCargoAuto;
import frc.robot.commands.OneCargoAuto;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunTower;
import frc.robot.commands.Shoot;
import frc.robot.commands.Taxi;
import frc.robot.commands.SysIdCommand;
import frc.robot.commands.ThreeCargoAuto;
import frc.robot.commands.TwoCargoAuto;
import frc.robot.commands.PrepareShooter.ShooterPreset;
import frc.robot.oi.HandheldOI;
import frc.robot.oi.OISelector;
import frc.robot.oi.OverrideOI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIORomi;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMAX;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.Alert;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedChoosers;
import frc.robot.util.SparkMAXBurnManager;
import frc.robot.util.Alert.AlertType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheels flywheels;
  private final Hood hood;
  private final Kicker kicker;
  private final Tower tower;
  private final Intake intake;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI() {};

  // Choosers
  private final LoggedChoosers choosers = new LoggedChoosers();
  private final Map<String, AutoRoutine> autoRoutineMap =
      new HashMap<String, AutoRoutine>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if flash should be burned
    SparkMAXBurnManager.update();

    // Instantiate subsystems
    if (Constants.getMode() == Mode.REPLAY) {
      drive = new Drive(new DriveIO() {});
      vision = new Vision(new VisionIO() {});
      flywheels = new Flywheels(new FlywheelsIO() {});
      hood = new Hood(new HoodIO() {});
      kicker = new Kicker(new KickerIO() {});
      tower = new Tower(new TowerIO() {});
      intake = new Intake(new IntakeIO() {});
    } else {
      switch (Constants.getRobot()) {
        case ROBOT_2022P:
          drive = new Drive(new DriveIOSparkMAX());
          vision = new Vision(new VisionIO() {});
          flywheels = new Flywheels(new FlywheelsIO() {});
          hood = new Hood(new HoodIO() {});
          kicker = new Kicker(new KickerIO() {});
          tower = new Tower(new TowerIO() {});
          intake = new Intake(new IntakeIOSparkMAX());
          break;
        case ROBOT_2020:
          drive = new Drive(new DriveIOSparkMAX());
          vision = new Vision(new VisionIOPhotonVision());
          flywheels = new Flywheels(new FlywheelsIO() {});
          hood = new Hood(new HoodIO() {});
          kicker = new Kicker(new KickerIO() {});
          tower = new Tower(new TowerIO() {});
          intake = new Intake(new IntakeIO() {});
          break;
        case ROBOT_KITBOT:
          drive = new Drive(new DriveIOTalonSRX());
          vision = new Vision(new VisionIO() {});
          flywheels = new Flywheels(new FlywheelsIO() {});
          hood = new Hood(new HoodIO() {});
          kicker = new Kicker(new KickerIO() {});
          tower = new Tower(new TowerIO() {});
          intake = new Intake(new IntakeIO() {});
          break;
        case ROBOT_SIMBOT:
          drive = new Drive(new DriveIOSim());
          vision = new Vision(new VisionIO() {});
          flywheels = new Flywheels(new FlywheelsIOSim());
          hood = new Hood(new HoodIO() {});
          kicker = new Kicker(new KickerIO() {});
          tower = new Tower(new TowerIO() {});
          intake = new Intake(new IntakeIO() {});
          break;
        case ROBOT_ROMI:
          drive = new Drive(new DriveIORomi());
          vision = new Vision(new VisionIO() {});
          flywheels = new Flywheels(new FlywheelsIO() {});
          hood = new Hood(new HoodIO() {});
          kicker = new Kicker(new KickerIO() {});
          tower = new Tower(new TowerIO() {});
          intake = new Intake(new IntakeIO() {});
          break;
        default:
          drive = new Drive(new DriveIO() {});
          vision = new Vision(new VisionIO() {});
          flywheels = new Flywheels(new FlywheelsIO() {});
          hood = new Hood(new HoodIO() {});
          kicker = new Kicker(new KickerIO() {});
          tower = new Tower(new TowerIO() {});
          intake = new Intake(new IntakeIO() {});
          break;
      }
    }

    // Set up subsystems
    drive.setOverrides(() -> overrideOI.getDriveDisable(),
        () -> overrideOI.getOpenLoop(), () -> overrideOI.getInternalEncoders());
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> choosers.getJoystickMode(), () -> handheldOI.getLeftDriveX(),
        () -> handheldOI.getLeftDriveY(), () -> handheldOI.getRightDriveX(),
        () -> handheldOI.getRightDriveY(),
        () -> handheldOI.getSniperModeButton().get()));
    vision.setOverrides(() -> overrideOI.getVisionLEDMode());
    vision.setTranslationConsumer(drive::addVisionMeasurement);
    tower.setDefaultCommand(
        new AutoIndex(tower, () -> overrideOI.getAutoIndexDisable()));

    // Set up auto routines
    autoRoutineMap.put("Do Nothing", new AutoRoutine(AutoPosition.ORIGIN,
        new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER)));

    autoRoutineMap.put("Five cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, new FiveCargoAuto(drive, vision,
            flywheels, hood, tower, kicker, intake)));
    autoRoutineMap.put("Four cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, new FourCargoAuto(drive, vision,
            flywheels, hood, tower, kicker, intake)));
    autoRoutineMap.put("Three cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, new ThreeCargoAuto(drive, vision,
            flywheels, hood, tower, kicker, intake)));

    autoRoutineMap.put("Two cargo (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A,
            new TwoCargoAuto(AutoPosition.TARMAC_A, drive, vision, flywheels,
                hood, tower, kicker, intake)));
    autoRoutineMap.put("Two cargo (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C,
            new TwoCargoAuto(AutoPosition.TARMAC_C, drive, vision, flywheels,
                hood, tower, kicker, intake)));
    autoRoutineMap.put("Two cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D,
            new TwoCargoAuto(AutoPosition.TARMAC_D, drive, vision, flywheels,
                hood, tower, kicker, intake)));

    autoRoutineMap.put("One cargo (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, new OneCargoAuto(false, drive,
            vision, flywheels, hood, tower, kicker)));
    autoRoutineMap.put("One cargo (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, new OneCargoAuto(false, drive,
            vision, flywheels, hood, tower, kicker)));
    autoRoutineMap.put("One cargo (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, new OneCargoAuto(false, drive,
            vision, flywheels, hood, tower, kicker)));
    autoRoutineMap.put("One cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, new OneCargoAuto(false, drive,
            vision, flywheels, hood, tower, kicker)));
    autoRoutineMap.put("One cargo (FA)", new AutoRoutine(AutoPosition.FENDER_A,
        new OneCargoAuto(true, drive, vision, flywheels, hood, tower, kicker)));
    autoRoutineMap.put("One cargo (FB)", new AutoRoutine(AutoPosition.FENDER_B,
        new OneCargoAuto(true, drive, vision, flywheels, hood, tower, kicker)));

    autoRoutineMap.put("Taxi (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, new Taxi(drive, true)));
    autoRoutineMap.put("Taxi (FB)",
        new AutoRoutine(AutoPosition.FENDER_B, new Taxi(drive, true)));

    autoRoutineMap.put("Run SysId (Drive)", new AutoRoutine(AutoPosition.ORIGIN,
        new SysIdCommand(drive, drive::driveVoltage, drive::getSysIdData)));
    autoRoutineMap.put("Run SysId (Big Flywheel)",
        new AutoRoutine(AutoPosition.ORIGIN,
            new SysIdCommand(flywheels,
                volts -> flywheels.runVoltage(volts, 0.0),
                flywheels::getBigSysIdData)));
    autoRoutineMap.put("Run SysId (Little Flywheel)",
        new AutoRoutine(AutoPosition.ORIGIN,
            new SysIdCommand(flywheels,
                volts -> flywheels.runVoltage(0.0, volts),
                flywheels::getLittleSysIdData)));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    // Instantiate OI classes and bind buttons
    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().clearButtons();
    overrideOI = OISelector.findOverrideOI();
    handheldOI = OISelector.findHandheldOI();

    // Bind new buttons
    handheldOI.getAutoAimButton().whileActiveOnce(new AutoAim(drive, vision));

    Trigger flywheelsReady = new Trigger(flywheels::atSetpoints);
    handheldOI.getShootButton().and(flywheelsReady)
        .whileActiveContinuous(new Shoot(tower, kicker));

    handheldOI.getIntakeExtendButton().whenActive(intake::extend, intake);
    handheldOI.getIntakeRetractButton().whenActive(intake::retract, intake);
    handheldOI.getIntakeForwardsButton()
        .whileActiveContinuous(new RunIntake(intake, true));
    handheldOI.getIntakeBackwardsButton()
        .whileActiveContinuous(new RunIntake(intake, false));

    Command lowerFenderCommand =
        new PrepareShooter(flywheels, hood, ShooterPreset.LOWER_FENDER);
    Command upperFenderCommand =
        new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_FENDER);
    Command upperTarmacCommand =
        new PrepareShooter(flywheels, hood, ShooterPreset.UPPER_TARMAC);

    handheldOI.getStartLowerFenderButton().whenActive(lowerFenderCommand);
    handheldOI.getStartUpperFenderButton().whenActive(upperFenderCommand);
    handheldOI.getStartUpperTarmacButton().whenActive(upperTarmacCommand);
    handheldOI.getStopFlywheelButton().cancelWhenActive(lowerFenderCommand)
        .cancelWhenActive(upperFenderCommand)
        .cancelWhenActive(upperTarmacCommand);

    handheldOI.getTowerUpButton()
        .whileActiveContinuous(new RunTower(tower, true));
    handheldOI.getTowerDownButton()
        .whileActiveContinuous(new RunTower(tower, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String routineString = choosers.getAutoRoutine();
    if (autoRoutineMap.containsKey(routineString)) {
      AutoRoutine routine = autoRoutineMap.get(routineString);
      drive.setPose(routine.position.getPose(), true);
      drive.resetOnNextVision();
      return routine.command;

    } else {
      DriverStation.reportError("Unknown auto routine: '" + routineString + "'",
          false);
      return null;
    }
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final Command command;

    public AutoRoutine(AutoPosition position, Command command) {
      this.position = position;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN, TARMAC_A, TARMAC_B, TARMAC_C, TARMAC_D, FENDER_A, FENDER_B;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        case TARMAC_A:
          return FieldConstants.referenceA
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.7));
        case TARMAC_B:
          return FieldConstants.referenceB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.2));
        case TARMAC_C:
          return FieldConstants.referenceC
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1));
        case TARMAC_D:
          return FieldConstants.referenceD
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.7));
        case FENDER_A:
          return FieldConstants.fenderA
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        case FENDER_B:
          return FieldConstants.fenderB
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        default:
          return new Pose2d();
      }
    }
  }
}
