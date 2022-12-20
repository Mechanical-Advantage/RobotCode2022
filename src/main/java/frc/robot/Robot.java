// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.BatteryTracker;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String batteryNameFile = "/home/lvuser/battery-name.txt";

  private RobotContainer robotContainer;
  private ByteLogReceiver logReceiver;
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private boolean batteryNameWritten = false;

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  private final Alert logOpenFileAlert =
      new Alert("Failed to open log file. Data will NOT be logged.", AlertType.ERROR);
  private final Alert logWriteAlert =
      new Alert("Failed write to the log file. Data will NOT be logged.", AlertType.ERROR);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", AlertType.WARNING);

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
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard/TunableNumbers");
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
          LoggedSystemStats.getInstance().setPowerDistributionConfig(50, ModuleType.kRev);
        }
        break;

      case SIM:
        logger.addDataReceiver(new LogSocketServer(5800));
        break;

      case REPLAY:
        String path = ByteLogReplay.promptForPath();
        logger.setReplaySource(new ByteLogReplay(path));
        logger.addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
        break;
    }
    logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Check for battery alert
    if (Constants.getMode() == Mode.REAL
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)) {
      File file = new File(batteryNameFile);
      if (file.exists()) {
        // Read previous battery name
        String previousBatteryName = "";
        try {
          previousBatteryName =
              new String(Files.readAllBytes(Paths.get(batteryNameFile)), StandardCharsets.UTF_8);
        } catch (IOException e) {
          e.printStackTrace();
        }

        if (previousBatteryName.equals(BatteryTracker.getName())) {
          // Same battery, set alert
          sameBatteryAlert.set(true);
          robotContainer.setSameBatteryAlert(true);
        } else {
          // New battery, delete file
          file.delete();
        }
      }
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();

    // Update LED mode
    robotContainer.updateLeds();

    // Log scheduled commands
    Logger.getInstance()
        .recordOutput(
            "ActiveCommands/Scheduler",
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
          System.out.println(
              String.format(
                  "*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
        } else {
          System.out.println(
              String.format(
                  "*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
        }
        autoMessagePrinted = true;
      }
    }

    // Write battery name if connected to field
    if (Constants.getMode() == Mode.REAL
        && !batteryNameWritten
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)
        && DriverStation.isFMSAttached()) {
      batteryNameWritten = true;
      try {
        FileWriter fileWriter = new FileWriter(batteryNameFile);
        fileWriter.write(BatteryTracker.getName());
        fileWriter.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    Threads.setCurrentThreadPriority(true, 10);
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
