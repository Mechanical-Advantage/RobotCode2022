/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

/**
 * A command that does a simple spin in place at a fixed power until the specified angle is reached.
 * Expect overshoot.
 */
public class TurnToAngleFast extends CommandBase {

  private final Drive drive;
  private final Rotation2d rotation;
  private final double power;

  private boolean spinLeft;

  /**
   * Creates a new TurnToAngleFast.
   */
  public TurnToAngleFast(Drive drive, Rotation2d rotation, double power) {
    addRequirements(drive);
    this.drive = drive;
    this.rotation = rotation;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double difference = drive.getRotation().minus(rotation).getDegrees();
    if (difference > 0) {
      drive.drivePercent(power, power * -1);
      spinLeft = false;
    } else {
      drive.drivePercent(power * -1, power);
      spinLeft = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double difference = drive.getRotation().minus(rotation).getDegrees();
    if (spinLeft) {
      if (difference > 0) {
        return true;
      }
    } else {
      if (difference < 0) {
        return true;
      }
    }
    return false;
  }
}
