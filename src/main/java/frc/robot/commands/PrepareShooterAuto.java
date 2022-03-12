// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.LinearInterpolation;
import frc.robot.util.PolynomialRegression;

public class PrepareShooterAuto extends CommandBase {
  private static final List<ShootingPosition> positionData =
      List.of(new ShootingPosition(0.0, 1000.0, 15.0));

  private static final PolynomialRegression flywheelSpeedRegression;
  private static final PolynomialRegression hoodAngleRegression;
  private static final LinearInterpolation towerSpeedInterpolation =
      new LinearInterpolation(List.of(new LinearInterpolation.Point(3.0, 0.35),
          new LinearInterpolation.Point(4.0, 0.6)));

  private final Flywheels flywheels;
  private final Hood hood;
  private final Tower tower;
  private final RobotState robotState;
  private final Translation2d staticPosition;

  static {
    // Create regressions based on position data
    double[] distances =
        positionData.stream().mapToDouble(x -> x.distanceMeters).toArray();
    double[] flywheelSpeeds =
        positionData.stream().mapToDouble(x -> x.flywheelSpeedRpm).toArray();
    double[] hoodAngles =
        positionData.stream().mapToDouble(x -> x.hoodAngleDegrees).toArray();
    flywheelSpeedRegression =
        new PolynomialRegression(distances, flywheelSpeeds, 2, "x");
    hoodAngleRegression =
        new PolynomialRegression(distances, hoodAngles, 2, "x");
  }

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on a known shooting position.
   */
  public PrepareShooterAuto(Flywheels flywheels, Hood hood, Tower tower,
      Translation2d position) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.tower = tower;
    this.robotState = null;
    this.staticPosition = position;
  }

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on odometry.
   */
  public PrepareShooterAuto(Flywheels flywheels, Hood hood, Tower tower,
      RobotState robotState) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.tower = tower;
    this.robotState = robotState;
    this.staticPosition = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (staticPosition != null) {
      update(staticPosition);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (robotState != null) {
      update(robotState.getLatestPose().getTranslation());
    }
    Logger.getInstance().recordOutput("ActiveCommands/PrepareShooterAuto",
        true);
  }

  public void update(Translation2d position) {
    double distance = position.getDistance(FieldConstants.hubCenter);
    flywheels.runVelocity(flywheelSpeedRegression.predict(distance));
    hood.moveToAngle(hoodAngleRegression.predict(distance));
    tower.requestShootPercent(towerSpeedInterpolation.predict(distance));
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

  private static class ShootingPosition {
    public final double distanceMeters;
    public final double flywheelSpeedRpm;
    public final double hoodAngleDegrees;

    public ShootingPosition(double distanceMeters, double flywheelSpeedRpm,
        double hoodAngleDegrees) {
      this.distanceMeters = distanceMeters;
      this.flywheelSpeedRpm = flywheelSpeedRpm;
      this.hoodAngleDegrees = hoodAngleDegrees;
    }
  }
}
