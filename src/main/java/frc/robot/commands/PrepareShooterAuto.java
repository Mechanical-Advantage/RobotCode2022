// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.PolynomialRegression;

public class PrepareShooterAuto extends CommandBase {

  private static final PolynomialRegression bigRegression =
      new PolynomialRegression(new double[] {0.0}, new double[] {0.0}, 2, "x");
  private static final PolynomialRegression littleRegression =
      new PolynomialRegression(new double[] {0.0}, new double[] {0.0}, 2, "x");

  private final Flywheels flywheels;
  private final Hood hood;
  private final Drive drive;
  private final Translation2d staticPosition;

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on a known shooting position.
   */
  public PrepareShooterAuto(Flywheels flywheels, Hood hood,
      Translation2d position) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.drive = null;
    this.staticPosition = position;
  }

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on odometry.
   */
  public PrepareShooterAuto(Flywheels flywheels, Hood hood, Drive drive) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.drive = drive;
    this.staticPosition = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setRaised(false);
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
    double bigSpeed = bigRegression.predict(distance);
    double littleSpeed = littleRegression.predict(distance);
    flywheels.runVelocity(bigSpeed, littleSpeed);
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
