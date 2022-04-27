// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.TreeMap;
import java.util.Map.Entry;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.Shoot;
import frc.robot.util.Alert;
import frc.robot.util.BatteryTracker;
import frc.robot.util.GeomUtil;
import frc.robot.util.Alert.AlertType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;
  private ByteLogReceiver logReceiver;
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.",
          AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.",
          AlertType.ERROR);
  private final Alert logOpenFileAlert = new Alert(
      "Failed to open log file. Data will NOT be logged", AlertType.ERROR);
  private final Alert logWriteAlert = new Alert(
      "Failed write to the log file. Data will NOT be logged", AlertType.ERROR);

  private final String zebraPath = "/path/to/file.csv";
  private final Transform2d zebraToCenter =
      new Transform2d(new Translation2d(0.2, 0.0), new Rotation2d());
  private TreeMap<Double, Translation2d> zebraData = new TreeMap<>();
  private Double matchStartTimestamp = null;

  public Robot() {
    super(Constants.loopPeriodSecs);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();
    setUseTiming(Constants.getMode() != Mode.REPLAY);
    LoggedNetworkTables.getInstance()
        .addTable("/SmartDashboard/TunableNumbers");
    logger.recordMetadata("Robot", Constants.getRobot().toString());
    logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
    logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
        String folder = Constants.logFolders.get(Constants.getRobot());
        if (folder != null) {
          logReceiver = new ByteLogReceiver(folder);
          logger.addDataReceiver(logReceiver);
        } else {
          logNoFileAlert.set(true);
        }
        logger.addDataReceiver(new LogSocketServer(5800));
        if (Constants.getRobot() == RobotType.ROBOT_2022C) {
          LoggedSystemStats.getInstance().setPowerDistributionConfig(50,
              ModuleType.kRev);
        }
        break;

      case SIM:
        logger.addDataReceiver(new LogSocketServer(5800));
        break;

      case REPLAY:
        String path = ByteLogReplay.promptForPath();
        logger.setReplaySource(new ByteLogReplay(path));
        logger.addDataReceiver(
            new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
        break;
    }
    logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Read zebra data from file
    String line = "";
    try (BufferedReader bufferedReader =
        new BufferedReader(new FileReader(zebraPath))) {
      while (true) {
        line = bufferedReader.readLine();
        if (line == null) {
          break;
        }
        String[] entryData = line.split(",");
        zebraData.put(Double.parseDouble(entryData[0]),
            new Translation2d(Double.parseDouble(entryData[1]),
                Double.parseDouble(entryData[2])));
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update LED mode
    robotContainer.updateLeds();

    // Log scheduled commands
    Logger.getInstance().recordOutput("ActiveCommands/Scheduler",
        NetworkTableInstance.getDefault()
            .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
            .getStringArray(new String[] {}));

    // Check logging faults
    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());
    if (logReceiver != null) {
      logOpenFileAlert.set(logReceiver.getOpenFault());
      logWriteAlert.set(logReceiver.getWriteFault());
    }

    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.println(String.format("*** Auto finished in %.2f secs ***",
              Timer.getFPGATimestamp() - autoStart));
        } else {
          System.out
              .println(String.format("*** Auto cancelled in %.2f secs ***",
                  Timer.getFPGATimestamp() - autoStart));
        }
        autoMessagePrinted = true;
      }
    }

    // Log zebra data
    if (matchStartTimestamp == null) {
      if (DriverStation.isEnabled()) {
        matchStartTimestamp = Timer.getFPGATimestamp();
      }
    }
    if (matchStartTimestamp != null) {
      double zebraTimestamp = Timer.getFPGATimestamp() - matchStartTimestamp;

      Entry<Double, Translation2d> zebraEntryFloor =
          zebraData.floorEntry(zebraTimestamp);
      Entry<Double, Translation2d> zebraEntryCeiling =
          zebraData.ceilingEntry(zebraTimestamp);
      if (zebraEntryFloor != null && zebraEntryCeiling != null) {
        Pose2d robotPose = robotContainer.getPose();
        Translation2d zebraTranslation = GeomUtil
            .interpolate(
                new Pose2d(zebraEntryFloor.getValue(), new Rotation2d()),
                new Pose2d(zebraEntryCeiling.getValue(), new Rotation2d()),
                (zebraTimestamp - zebraEntryFloor.getKey())
                    / (zebraEntryCeiling.getKey() - zebraEntryFloor.getKey()))
            .getTranslation();

        // Convert to meters
        zebraTranslation =
            new Translation2d(Units.feetToMeters(zebraTranslation.getX()),
                Units.feetToMeters(zebraTranslation.getY()));

        // Flip pose for blue alliance (needs to be relative to own driver station wall)
        if (DriverStation.getAlliance() == Alliance.Blue) {
          zebraTranslation = new Translation2d(FieldConstants.fieldLength,
              FieldConstants.fieldWidth).minus(zebraTranslation);
        }

        // Find pose of robot center
        Pose2d zebraPose = new Pose2d(zebraTranslation, robotPose.getRotation())
            .transformBy(zebraToCenter);

        Logger.getInstance().recordOutput("Zebra/Pose",
            new double[] {zebraPose.getX(), zebraPose.getY(),
                zebraPose.getRotation().getRadians()});

        double errorMeters = zebraPose.getTranslation()
            .getDistance(robotContainer.getPose().getTranslation());
        Logger.getInstance().recordOutput("Zebra/ErrorMetersAll", errorMeters);
        if (!robotContainer.isClimbMode()) {
          Logger.getInstance().recordOutput("Zebra/ErrorMetersNonClimb",
              errorMeters);
        }
        if (Shoot.isActive()) {
          Logger.getInstance().recordOutput("Zebra/ErrorMetersShooting",
              errorMeters);
        }
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    robotContainer.updateOI();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    robotContainer.resetClimber();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
