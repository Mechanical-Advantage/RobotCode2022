// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.LinearInterpolation;

public class PrepareShooterAuto extends CommandBase {

  private static final LinearInterpolation lowerInterpolation =
      new LinearInterpolation(List.of(new LinearInterpolation.Point(1.4, 500.0),
          new LinearInterpolation.Point(2.8, 900.0)));
  private static final LinearInterpolation upperInterpolation =
      new LinearInterpolation(
          List.of(new LinearInterpolation.Point(1.0, 1100.0),
              new LinearInterpolation.Point(2.0, 1175.0)));

  private final boolean upper;
  private final Flywheels flywheels;
  private final Hood hood;
  private final Drive drive;
  private final Translation2d staticPosition;

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on a known shooting position.
   */
  public PrepareShooterAuto(boolean upper, Flywheels flywheels, Hood hood,
      Translation2d position) {
    addRequirements(flywheels);
    this.upper = upper;
    this.flywheels = flywheels;
    this.hood = hood;
    this.drive = null;
    this.staticPosition = position;
  }

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on odometry.
   */
  public PrepareShooterAuto(boolean upper, Flywheels flywheels, Hood hood,
      Drive drive) {
    addRequirements(flywheels);
    this.upper = upper;
    this.flywheels = flywheels;
    this.hood = hood;
    this.drive = drive;
    this.staticPosition = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.requestShootPosition(!upper);
    if (staticPosition != null) {
      update(staticPosition);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drive != null) {
      update(drive.getPose().getTranslation());
    }
    Logger.getInstance().recordOutput("ActiveCommands/PrepareShooterAuto",
        true);
  }

  public void update(Translation2d position) {
    double distance = position.getDistance(FieldConstants.hubCenter);
    double speed = upper ? upperInterpolation.predict(distance)
        : lowerInterpolation.predict(distance);
    flywheels.runVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheels.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
