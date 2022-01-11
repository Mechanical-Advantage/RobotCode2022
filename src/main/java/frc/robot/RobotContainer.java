// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.SysIdCommand;
import frc.robot.oi.HandheldOI;
import frc.robot.oi.OISelector;
import frc.robot.oi.OverrideOI;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.DriveTrainIO;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final DriveTrain driveTrain;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI() {};

  // Dashboard choosers
  private final SendableChooser<Command> autoChooser =
      new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate subsystems
    if (Constants.getMode() == Mode.REPLAY) {
      driveTrain = new DriveTrain(new DriveTrainIO() {});
    } else {
      switch (Constants.getRobot()) {
        default:
          driveTrain = new DriveTrain(new DriveTrainIO() {});
          break;

      }
    }

    // Set up subsystems
    driveTrain.setOverrides(() -> overrideOI.getDriveDisable(),
        () -> overrideOI.getOpenLoop());
    driveTrain.setDefaultCommand(new DriveWithJoysticks(driveTrain,
        () -> handheldOI.getLeftDriveX(), () -> handheldOI.getLeftDriveY(),
        () -> handheldOI.getRightDriveX(), () -> handheldOI.getRightDriveY()));

    // Set up auto chooser
    autoChooser.setDefaultOption("Do Nothing", null);
    autoChooser.addOption("Run SysId (DriveTrain)", new SysIdCommand(driveTrain,
        driveTrain::driveVoltage, driveTrain::getSysIdData));
    SmartDashboard.putData("Auto Mode", autoChooser);

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
    return autoChooser.getSelected();
  }
}
