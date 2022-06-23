// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimSimple;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.IdleDuck;
import frc.robot.commands.PlayDuckSound;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterizationStepped;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.PrepareShooterPreset;
import frc.robot.commands.ResetHood;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TrackWidthCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.commands.RunIntake.IntakeMode;
import frc.robot.commands.autos.DuckToHangar;
import frc.robot.commands.autos.FiveCargoAuto;
import frc.robot.commands.autos.FourCargoAuto;
import frc.robot.commands.autos.FourCargoAutoAvoidD;
import frc.robot.commands.autos.FourCargoAutoCross;
import frc.robot.commands.autos.HPPractice;
import frc.robot.commands.autos.OneCargoAuto;
import frc.robot.commands.autos.PartnerAuto;
import frc.robot.commands.autos.Spin;
import frc.robot.commands.autos.Taxi;
import frc.robot.commands.autos.ThreeCargoAuto;
import frc.robot.commands.autos.ThreeCargoAutoCrossMidline;
import frc.robot.commands.autos.TwoCargoAuto;
import frc.robot.commands.autos.TwoCargoAutoAndEject;
import frc.robot.commands.climber.AutoClimb;
import frc.robot.commands.climber.ResetClimber;
import frc.robot.commands.climber.RunClimberWithJoystick;
import frc.robot.commands.climber.RunClimberToPosition;
import frc.robot.oi.HandheldOI;
import frc.robot.oi.OISelector;
import frc.robot.oi.OverrideOI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMAX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIORomi;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMAX;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.duck.Duck;
import frc.robot.subsystems.duck.DuckIO;
import frc.robot.subsystems.duck.DuckIOVictorSPX;
import frc.robot.subsystems.duck.Duck.DuckSound;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSparkMAX;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.flywheels.FlywheelsIOSparkMAX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSparkMAX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIORio;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.pneumatics.PneumaticsIO;
import frc.robot.subsystems.pneumatics.PneumaticsIOCTRE;
import frc.robot.subsystems.pneumatics.PneumaticsIOREV;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Alert;
import frc.robot.util.DisabledInstantCommand;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedChoosers;
import frc.robot.util.SparkMAXBurnManager;
import frc.robot.util.StickyTrigger;
import frc.robot.util.UninterruptibleScheduleCommand;
import frc.robot.util.Alert.AlertType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private RobotState robotState = new RobotState();

  // Subsystems
  private Drive drive;
  private Vision vision;
  private Flywheels flywheels;
  private Hood hood;
  private Feeder feeder;
  private Intake intake;
  private Climber climber;
  private Pneumatics pneumatics;
  private Leds leds;
  private Duck duck;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI() {};

  // Choosers
  private final LoggedChoosers choosers = new LoggedChoosers();
  private final Map<String, AutoRoutine> autoRoutineMap =
      new HashMap<String, AutoRoutine>();

  private boolean climberResetComplete = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if flash should be burned
    SparkMAXBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2022C:
          drive = new Drive(new DriveIOSparkMAX());
          vision = new Vision(new VisionIOLimelight());
          flywheels = new Flywheels(new FlywheelsIOSparkMAX());
          hood = new Hood(new HoodIOSparkMAX());
          feeder = new Feeder(new FeederIOSparkMAX());
          intake = new Intake(new IntakeIOSparkMAX());
          climber = new Climber(new ClimberIOSparkMAX());
          pneumatics = new Pneumatics(new PneumaticsIOREV());
          leds = new Leds(new LedsIORio());
          break;
        case ROBOT_2022P:
          drive = new Drive(new DriveIOSparkMAX());
          flywheels = new Flywheels(new FlywheelsIOSim());
          duck = new Duck(new DuckIOVictorSPX());
          break;
        case ROBOT_2020:
          drive = new Drive(new DriveIOSparkMAX());
          pneumatics = new Pneumatics(new PneumaticsIOCTRE());
          break;
        case ROBOT_KITBOT:
          drive = new Drive(new DriveIOTalonSRX());
          break;
        case ROBOT_SIMBOT:
          drive = new Drive(new DriveIOSim());
          flywheels = new Flywheels(new FlywheelsIOSim());
          hood = new Hood(new HoodIOSim());
          duck = new Duck(new DuckIOVictorSPX());
          break;
        case ROBOT_ROMI:
          drive = new Drive(new DriveIORomi());
          break;
      }
    }

    // Instantiate missing subsystems
    drive = drive != null ? drive : new Drive(new DriveIO() {});
    vision = vision != null ? vision : new Vision(new VisionIO() {});
    flywheels =
        flywheels != null ? flywheels : new Flywheels(new FlywheelsIO() {});
    hood = hood != null ? hood : new Hood(new HoodIO() {});
    feeder = feeder != null ? feeder : new Feeder(new FeederIO() {});
    intake = intake != null ? intake : new Intake(new IntakeIO() {});
    climber = climber != null ? climber : new Climber(new ClimberIO() {});
    pneumatics =
        pneumatics != null ? pneumatics : new Pneumatics(new PneumaticsIO() {});
    leds = leds != null ? leds : new Leds(new LedsIO() {});
    duck = duck != null ? duck : new Duck(new DuckIO() {});

    // Set up subsystems
    robotState.setLeds(leds);
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> choosers.getJoystickMode(), () -> handheldOI.getLeftDriveX(),
        () -> handheldOI.getLeftDriveY(), () -> handheldOI.getRightDriveX(),
        () -> handheldOI.getRightDriveY(),
        () -> handheldOI.getSniperModeButton().get()));
    drive.setSuppliers(() -> overrideOI.getDriveDisable(),
        () -> overrideOI.getOpenLoop(), () -> overrideOI.getInternalEncoders());
    drive.setRobotState(robotState);
    drive.setLeds(leds);
    vision
        .setDefaultCommand(new RunCommand(() -> vision.setPipeline(0), vision));
    vision.setSuppliers(() -> overrideOI.getVisionLedMode(),
        () -> overrideOI.getClimbMode());
    vision.setRobotState(robotState);
    flywheels.setLeds(leds);
    hood.setDefaultCommand(new RunCommand(hood::moveToIdle, hood));
    hood.setRobotState(robotState);
    feeder.setSubsystems(leds, intake, flywheels, hood);
    feeder.setOverride(() -> overrideOI.getCargoSensorDisable());
    pneumatics.setSupplier(() -> overrideOI.getClimbMode());
    duck.setDefaultCommand(new IdleDuck(duck, drive));

    // Set up auto routines
    autoRoutineMap.put("Do Nothing",
        new AutoRoutine(AutoPosition.ORIGIN, true, new InstantCommand()));

    autoRoutineMap.put("Duck to hangar (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, false,
            new DuckToHangar(false, drive, robotState)));
    autoRoutineMap.put("Duck to hangar (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, false,
            new DuckToHangar(true, drive, robotState)));
    autoRoutineMap.put("Short duck taxi (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, false,
            new Taxi(drive, false).andThen(new Spin(drive))));
    autoRoutineMap.put("Long duck taxi (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, false,
            new Taxi(drive, true).andThen(new Spin(drive))));

    autoRoutineMap.put("Five cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false, new FiveCargoAuto(
            robotState, drive, vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Four cargo, standard (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false, new FourCargoAuto(
            robotState, drive, vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Four cargo, cross field (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false, new FourCargoAutoCross(
            robotState, drive, vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Four cargo, avoid tarmac D (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, false, new FourCargoAutoAvoidD(
            robotState, drive, vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Three cargo, standard (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false, new ThreeCargoAuto(
            robotState, drive, vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Three cargo, cross midline (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false,
            new ThreeCargoAutoCrossMidline(robotState, drive, vision, flywheels,
                hood, feeder, intake, leds)));
    autoRoutineMap.put("Three cargo, collect partner (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, false, new PartnerAuto(
            robotState, drive, vision, flywheels, hood, feeder, intake, leds)));

    autoRoutineMap.put("Two cargo, eject hangar (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false,
            new TwoCargoAutoAndEject(true, robotState, drive, vision, flywheels,
                hood, feeder, intake, leds)));
    autoRoutineMap.put("Two cargo, eject fender (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false,
            new TwoCargoAutoAndEject(false, robotState, drive, vision,
                flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Two cargo (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, false,
            new TwoCargoAuto(true, AutoPosition.TARMAC_A, robotState, drive,
                vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Two cargo (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, false,
            new TwoCargoAuto(true, AutoPosition.TARMAC_C, robotState, drive,
                vision, flywheels, hood, feeder, intake, leds)));
    autoRoutineMap.put("Two cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, false,
            new TwoCargoAuto(true, AutoPosition.TARMAC_D, robotState, drive,
                vision, flywheels, hood, feeder, intake, leds)));

    autoRoutineMap.put("One cargo (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, true, new OneCargoAuto(false,
            robotState, drive, vision, flywheels, hood, feeder, leds)));
    autoRoutineMap.put("One cargo (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, true, new OneCargoAuto(false,
            robotState, drive, vision, flywheels, hood, feeder, leds)));
    autoRoutineMap.put("One cargo (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, true, new OneCargoAuto(false,
            robotState, drive, vision, flywheels, hood, feeder, leds)));
    autoRoutineMap.put("One cargo (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, true, new OneCargoAuto(false,
            robotState, drive, vision, flywheels, hood, feeder, leds)));
    autoRoutineMap.put("One cargo (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, true, new OneCargoAuto(true,
            robotState, drive, vision, flywheels, hood, feeder, leds)));
    autoRoutineMap.put("One cargo (FB)",
        new AutoRoutine(AutoPosition.FENDER_B, true, new OneCargoAuto(true,
            robotState, drive, vision, flywheels, hood, feeder, leds)));

    autoRoutineMap.put("Taxi (TA)",
        new AutoRoutine(AutoPosition.TARMAC_A, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TB)",
        new AutoRoutine(AutoPosition.TARMAC_B, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TC)",
        new AutoRoutine(AutoPosition.TARMAC_C, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (TD)",
        new AutoRoutine(AutoPosition.TARMAC_D, true, new Taxi(drive, false)));
    autoRoutineMap.put("Taxi (FA)",
        new AutoRoutine(AutoPosition.FENDER_A, true, new Taxi(drive, true)));
    autoRoutineMap.put("Taxi (FB)",
        new AutoRoutine(AutoPosition.FENDER_B, true, new Taxi(drive, true)));

    autoRoutineMap.put("HP Practice", new AutoRoutine(AutoPosition.ORIGIN,
        false, new HPPractice(robotState, drive, intake, feeder, leds)));

    autoRoutineMap.put("Track Width Characterization",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new TrackWidthCharacterization(drive, drive::driveVoltage,
                drive::getLeftPositionMeters, drive::getRightPositionMeters,
                drive::getCharacterizationGyroPosition)));

    FeedForwardCharacterizationData driveLeftData =
        new FeedForwardCharacterizationData("Drive/Left");
    FeedForwardCharacterizationData driveRightData =
        new FeedForwardCharacterizationData("Drive/Right");
    autoRoutineMap.put("FF Characterization (Drive/Forwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterization(drive, true, driveLeftData,
                driveRightData, drive::driveVoltage,
                drive::getCharacterizationVelocityLeft,
                drive::getCharacterizationVelocityRight)));
    autoRoutineMap.put("FF Characterization (Drive/Backwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterization(drive, false, driveLeftData,
                driveRightData, drive::driveVoltage,
                drive::getCharacterizationVelocityLeft,
                drive::getCharacterizationVelocityRight)));

    FeedForwardCharacterizationData flywheelData =
        new FeedForwardCharacterizationData("Flywheels");
    autoRoutineMap.put("FF Characterization (Flywheels/Forwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterizationStepped(flywheels, true,
                flywheelData, volts -> flywheels.runVoltage(volts),
                flywheels::getCharacterizationVelocity,
                flywheels::getCharacterizationAppliedVolts)));
    autoRoutineMap.put("FF Characterization (Flywheels/Backwards)",
        new AutoRoutine(AutoPosition.ORIGIN, false,
            new FeedForwardCharacterizationStepped(flywheels, false,
                flywheelData, volts -> flywheels.runVoltage(volts),
                flywheels::getCharacterizationVelocity,
                flywheels::getCharacterizationAppliedVolts)));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    // Set up camera for practice bot
    if (Constants.getMode() == Mode.REAL
        && Constants.getRobot() == RobotType.ROBOT_2022P) {
      UsbCamera camera = CameraServer.startAutomaticCapture("Driver Camera", 0);
      camera.setResolution(160, 120);
      camera.setFPS(15);
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

    handheldOI.getLogMarkerButton().whileActiveContinuous(new RunCommand(
        () -> Logger.getInstance().recordOutput("Marker", true)));

    // *** DRIVER CONTROLS ***
    handheldOI.getAutoDriveButton().whileActiveContinuous(new DriveToTarget(
        drive, robotState, vision, handheldOI::getLeftDriveY));

    Trigger simpleAutoAim = new Trigger(overrideOI::getSimpleAutoAim);
    handheldOI.getAutoAimButton().and(simpleAutoAim.negate()).whileActiveOnce(
        new AutoAim(drive, robotState, vision, null, handheldOI::getLeftDriveY)
            .perpetually());
    handheldOI.getAutoAimButton().and(simpleAutoAim).whileActiveOnce(
        new AutoAimSimple(drive, vision, false, handheldOI::getLeftDriveY)
            .perpetually());

    Command hoodResetSequence = new SequentialCommandGroup(new WaitCommand(1.5),
        new UninterruptibleScheduleCommand(new ResetHood(hood)));
    Trigger flywheelsReady = new Trigger(flywheels::atGoal);
    Trigger hoodReady = new Trigger(hood::atGoal);
    handheldOI.getShootButton().and(flywheelsReady).and(hoodReady)
        .whileActiveContinuous(
            new Shoot(feeder, leds, handheldOI::setDriverRumble))
        .whenInactive(hoodResetSequence).cancelWhenActive(hoodResetSequence);

    // *** OPERATOR CONTROLS ***
    Trigger climbMode = new Trigger(overrideOI::getClimbMode);
    Trigger normalMode = climbMode.negate();

    new Trigger(DriverStation::isTeleopEnabled).whenActive(intake::retract,
        intake);
    handheldOI.getIntakeForwardsExtendButton()
        .or(handheldOI.getIntakeBackwardsExtendButton()).and(normalMode)
        .whenActive(intake::extend, intake)
        .whenInactive(intake::retract, intake);
    handheldOI.getIntakeForwardsRunButton().and(normalMode)
        .whileActiveContinuous(new RunIntake(IntakeMode.FORWARDS, intake,
            feeder, leds, handheldOI::setOperatorRumble));
    handheldOI.getIntakeBackwardsRunButton().and(normalMode)
        .whileActiveContinuous(new RunIntake(IntakeMode.BACKWARDS, intake,
            feeder, leds, handheldOI::setOperatorRumble));

    StickyTrigger flywheelFenderTrigger = new StickyTrigger();
    flywheelFenderTrigger.whileActiveContinuous(new PrepareShooterPreset(
        flywheels, hood, feeder, ShooterPreset.UPPER_FENDER));
    StickyTrigger flywheelTarmacTrigger = new StickyTrigger();
    flywheelTarmacTrigger.whileActiveContinuous(new PrepareShooterPreset(
        flywheels, hood, feeder, ShooterPreset.UPPER_TARMAC));
    StickyTrigger flywheelLaunchpadTrigger = new StickyTrigger();
    flywheelLaunchpadTrigger.whileActiveContinuous(new PrepareShooterPreset(
        flywheels, hood, feeder, ShooterPreset.UPPER_LAUNCHPAD));
    StickyTrigger flywheelAutoTrigger = new StickyTrigger();
    flywheelAutoTrigger.whileActiveContinuous(
        new PrepareShooterAuto(flywheels, hood, feeder, robotState));

    Trigger usePresets = new Trigger(overrideOI::getShootPresets);
    handheldOI.getStartFlywheelFenderButton().and(normalMode)
        .whenActive(flywheelFenderTrigger::setActive) // Activate only requested command
        .whenActive(flywheelTarmacTrigger::setInactive)
        .whenActive(flywheelLaunchpadTrigger::setInactive)
        .whenActive(flywheelAutoTrigger::setInactive);
    handheldOI.getStartFlywheelTarmacButton().and(normalMode).and(usePresets)
        .whenActive(flywheelFenderTrigger::setInactive)
        .whenActive(flywheelTarmacTrigger::setActive) // Activate only requested command
        .whenActive(flywheelLaunchpadTrigger::setInactive)
        .whenActive(flywheelAutoTrigger::setInactive);
    handheldOI.getStartFlywheelLaunchpadButton().and(normalMode)
        .whenActive(flywheelFenderTrigger::setInactive)
        .whenActive(flywheelTarmacTrigger::setInactive)
        .whenActive(flywheelLaunchpadTrigger::setActive) // Activate only requested command
        .whenActive(flywheelAutoTrigger::setInactive);
    handheldOI.getStartFlywheelAutoButton().and(normalMode)
        .and(usePresets.negate()).whenActive(flywheelFenderTrigger::setInactive)
        .whenActive(flywheelTarmacTrigger::setInactive)
        .whenActive(flywheelLaunchpadTrigger::setInactive)
        .whenActive(flywheelAutoTrigger::setActive); // Activate only requested command
    handheldOI.getStopFlywheelButton()
        .or(new Trigger(DriverStation::isDisabled)).and(normalMode)
        .whenActive(flywheelFenderTrigger::setInactive)
        .whenActive(flywheelTarmacTrigger::setInactive)
        .whenActive(flywheelLaunchpadTrigger::setInactive)
        .whenActive(flywheelAutoTrigger::setInactive);

    handheldOI.getShooterIncrement()
        .whenActive(new DisabledInstantCommand(flywheels::incrementOffset));
    handheldOI.getShooterDecrement()
        .whenActive(new DisabledInstantCommand(flywheels::decrementOffset));

    handheldOI.getDuckSoundButton1()
        .whenActive(new PlayDuckSound(duck, DuckSound.QUACK_1));
    handheldOI.getDuckSoundButton2()
        .whenActive(new PlayDuckSound(duck, DuckSound.QUACK_2));
    handheldOI.getDuckSoundButton3()
        .whenActive(new PlayDuckSound(duck, DuckSound.QUACK_3));
    handheldOI.getDuckSoundButton4()
        .whenActive(new PlayDuckSound(duck, DuckSound.QUACK_4));
    handheldOI.getDuckSoundButton5()
        .whenActive(new PlayDuckSound(duck, DuckSound.QUACK_5));

    // *** CLIMB CONTROLS ***
    climbMode.whileActiveContinuous(new StartEndCommand(
        () -> leds.setClimbing(true), () -> leds.setClimbing(false)));
    climbMode.whileActiveContinuous(new RunClimberWithJoystick(climber,
        handheldOI::getClimbStick, overrideOI::getClimbOpenLoop));
    climbMode.whenActive(flywheelFenderTrigger::setInactive)
        .whenActive(flywheelTarmacTrigger::setInactive)
        .whenActive(flywheelLaunchpadTrigger::setInactive)
        .whenActive(flywheelAutoTrigger::setInactive);
    climbMode.whenActive(intake::retract, intake);
    climbMode
        .whileActiveContinuous(new RunCommand(() -> hood.moveToBottom(), hood));
    climbMode.whenInactive(() -> climber.runVoltage(0.0), climber);

    Trigger climbClosedLoop =
        new Trigger(overrideOI::getClimbOpenLoop).negate();
    handheldOI.getClimbTop().and(climbMode).and(climbClosedLoop)
        .toggleWhenActive(new RunClimberToPosition(climber, true), false);
    handheldOI.getClimbBottom().and(climbMode).and(climbClosedLoop)
        .toggleWhenActive(new RunClimberToPosition(climber, false), false);
    handheldOI.getClimbAuto().and(climbMode).and(climbClosedLoop)
        .toggleWhenActive(new AutoClimb(climber, drive, leds), false);
    handheldOI.getClimbReset().and(climbMode)
        .toggleWhenActive(new ResetClimber(climber), false);
  }

  /** Called when the robot is enabled to begin resetting the climber. */
  public void resetClimber() {
    if (!climberResetComplete) {
      new ResetClimber(climber).schedule(false);
      climberResetComplete = true;
    }
  }

  /** Updates the LED mode periodically. */
  public void updateLeds() {
    leds.update();
  }

  /** Called at the start of auto to schedule the match start quack. */
  public void scheduleMatchStartQuack() {
    new WaitCommand(2.5).andThen(new PlayDuckSound(duck, DuckSound.MATCH_START))
        .schedule();
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
      robotState.resetPose(routine.position.getPose());
      if (routine.useVision) {
        robotState.resetOnNextVision();
      }
      vision.setAutoEnabled(routine.useVision);
      return routine.command;

    } else {
      DriverStation.reportError("Unknown auto routine: '" + routineString + "'",
          false);
      return null;
    }
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final boolean useVision;
    public final Command command;

    public AutoRoutine(AutoPosition position, boolean useVision,
        Command command) {
      this.position = position;
      this.useVision = useVision;
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
