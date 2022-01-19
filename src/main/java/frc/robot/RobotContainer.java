// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.SysIdCommand;
import frc.robot.oi.HandheldOI;
import frc.robot.oi.OISelector;
import frc.robot.oi.OverrideOI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMAX;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.util.BatteryTracker;
import frc.robot.util.LoggedChoosers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI() {};

  // Choosers
  private final LoggedChoosers choosers = new LoggedChoosers();
  private final Map<String, Pose2d> autoPositionMap =
      new HashMap<String, Pose2d>();
  private final Map<String, Command> autoRoutineMap =
      new HashMap<String, Command>();

  // Battery tracker
  private final BatteryTracker batteryTracker = new BatteryTracker();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Look for battery for up to 1 second
    batteryTracker.scanBattery(1.0);

    // Instantiate subsystems
    if (Constants.getMode() == Mode.REPLAY) {
      drive = new Drive(new DriveIO() {});
    } else {
      switch (Constants.getRobot()) {
        case ROBOT_2020:
          drive = new Drive(new DriveIOSparkMAX());
          break;
        case ROBOT_KITBOT:
          drive = new Drive(new DriveIOTalonSRX());
          break;
        case ROBOT_SIMBOT:
          drive = new Drive(new DriveIOSim());
          break;
        default:
          drive = new Drive(new DriveIO() {});
          break;

      }
    }

    // Set up subsystems
    drive.setOverrides(() -> overrideOI.getDriveDisable(),
        () -> overrideOI.getOpenLoop());
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> choosers.getJoystickMode(), () -> handheldOI.getLeftDriveX(),
        () -> handheldOI.getLeftDriveY(), () -> handheldOI.getRightDriveX(),
        () -> handheldOI.getRightDriveY()));

    // Set up auto positions
    Transform2d autoPositionTransformLeft = new Transform2d(
        new Translation2d(-0.5, FieldConstants.tarmacMissingSideLength / 2),
        Rotation2d.fromDegrees(180));
    Transform2d autoPositionTransformRight = new Transform2d(
        new Translation2d(-0.5, -FieldConstants.tarmacMissingSideLength / 2),
        Rotation2d.fromDegrees(180));
    autoPositionMap.put("Origin", new Pose2d());
    autoPositionMap.put("Tarmac A",
        FieldConstants.referenceA.transformBy(autoPositionTransformLeft));
    autoPositionMap.put("Tarmac B",
        FieldConstants.referenceB.transformBy(autoPositionTransformRight));
    autoPositionMap.put("Tarmac C",
        FieldConstants.referenceC.transformBy(autoPositionTransformLeft));
    autoPositionMap.put("Tarmac D",
        FieldConstants.referenceD.transformBy(autoPositionTransformRight));

    // Set up auto routines
    autoRoutineMap.put("Do Nothing", null);
    autoRoutineMap.put("Test Motion Profile",
        new MotionProfileCommand(drive, 0.0,
            List.of(new Pose2d(), new Pose2d(3.0, 1.0, new Rotation2d())), 0.0,
            false));
    autoRoutineMap.put("Run SysId (Drive)",
        new SysIdCommand(drive, drive::driveVoltage, drive::getSysIdData));

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
    // handheldOI.getAutoAimButton()
    // .whenActive(new PrintCommand("Activating the auto aim!"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String positionString = choosers.getAutoPosition();
    if (autoPositionMap.containsKey(positionString)) {
      drive.setPose(autoPositionMap.get(positionString));
    } else {
      DriverStation.reportError(
          "Unknown auto position: '" + positionString + "'", false);
    }

    String routineString = choosers.getAutoRoutine();
    if (autoRoutineMap.containsKey(routineString)) {
      return autoRoutineMap.get(routineString);
    } else {
      DriverStation.reportError("Unknown auto routine: '" + routineString + "'",
          false);
      return null;
    }
  }
}
