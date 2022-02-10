// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TwoPointPoseController;

public class DriveToPoint extends CommandBase {

  private final Drive drive;
  private final TwoPointPoseController controller;
  private final DifferentialDriveKinematics kinematics;

  /** Creates a new DriveToPoint. */
  public DriveToPoint(Drive drive, Pose2d target) {
    addRequirements(drive);
    this.drive = drive;

    switch (Constants.getRobot()) {
      case ROBOT_2022P:
        controller =
            new TwoPointPoseController(new PIDController(6.0, 0.0, 0.0), 0.05,
                new PIDController(3.0, 0.0, 0.0), 0.15);
        break;
      default:
        controller =
            new TwoPointPoseController(new PIDController(0.0, 0.0, 0.0), 0.0,
                new PIDController(0.0, 0.0, 0.0), 0.0);
    }
    controller.setTarget(target);
    kinematics = new DifferentialDriveKinematics(drive.getTrackWidthMeters());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = controller.calculate(drive.getPose());
    DifferentialDriveWheelSpeeds differentialSpeeds =
        kinematics.toWheelSpeeds(chassisSpeeds);
    drive.driveVelocity(differentialSpeeds.leftMetersPerSecond,
        differentialSpeeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atTarget();
  }
}
