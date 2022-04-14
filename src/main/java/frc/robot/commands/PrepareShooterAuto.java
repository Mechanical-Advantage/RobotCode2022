// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.LinearInterpolation;

public class PrepareShooterAuto extends CommandBase {
  private static final List<ShootingPosition> positionData =
      List.of(new ShootingPosition(53.0, 1220.0, 3.0),
          new ShootingPosition(64.0, 1170.0, 11.0),
          new ShootingPosition(89.0, 1170.0, 17.0),
          new ShootingPosition(113.0, 1180.0, 21.0),
          new ShootingPosition(123.0, 1190.0, 24.0),
          new ShootingPosition(148.0, 1260.0, 26.0),
          new ShootingPosition(187.0, 1370.0, 31.0),
          new ShootingPosition(220.0, 1500.0, 31.0),
          new ShootingPosition(240.0, 1560.0, 31.0),
          new ShootingPosition(260.0, 1670.0, 31.0),
          new ShootingPosition(280.0, 1800.0, 31.0));

  private static final LinearInterpolation flywheelSpeedInterpolation;
  private static final LinearInterpolation hoodAngleInterpolation;
  private static final LinearInterpolation towerSpeedInterpolation =
      new LinearInterpolation(List.of(
          new LinearInterpolation.Point(Units.inchesToMeters(60.0), 0.35),
          new LinearInterpolation.Point(Units.inchesToMeters(80.0), 0.6)));

  private final Flywheels flywheels;
  private final Hood hood;
  private final Feeder feeder;
  private final RobotState robotState;
  private final Translation2d staticPosition;

  static {
    // Create interpolations based on position data
    List<LinearInterpolation.Point> flywheelSpeedPoints = new ArrayList<>();
    List<LinearInterpolation.Point> hoodAnglePoints = new ArrayList<>();
    for (ShootingPosition position : positionData) {
      flywheelSpeedPoints.add(new LinearInterpolation.Point(
          Units.inchesToMeters(position.distanceInches),
          position.flywheelSpeedRpm));
      hoodAnglePoints.add(new LinearInterpolation.Point(
          Units.inchesToMeters(position.distanceInches),
          position.hoodAngleDegrees));
    }
    flywheelSpeedInterpolation = new LinearInterpolation(flywheelSpeedPoints);
    hoodAngleInterpolation = new LinearInterpolation(hoodAnglePoints);
  }

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on a known shooting position. Set the feeder to null to disable controlling the
   * tower speed.
   */
  public PrepareShooterAuto(Flywheels flywheels, Hood hood, Feeder feeder,
      Translation2d position) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.feeder = feeder;
    this.robotState = null;
    this.staticPosition = position;
  }

  /**
   * Creates a new PrepareShooterAuto. Runs the flywheel and sets the hood position for the upper
   * shot based on odometry. Set the feeder to null to disable controlling the tower speed.
   */
  public PrepareShooterAuto(Flywheels flywheels, Hood hood, Feeder feeder,
      RobotState robotState) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.feeder = feeder;
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
    flywheels.runVelocity(flywheelSpeedInterpolation.predict(distance));
    hood.moveToAngle(hoodAngleInterpolation.predict(distance));
    if (feeder != null) {
      feeder
          .requestTowerShootPercent(towerSpeedInterpolation.predict(distance));
    }
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
    public final double distanceInches;
    public final double flywheelSpeedRpm;
    public final double hoodAngleDegrees;

    public ShootingPosition(double distanceInches, double flywheelSpeedRpm,
        double hoodAngleDegrees) {
      this.distanceInches = distanceInches;
      this.flywheelSpeedRpm = flywheelSpeedRpm;
      this.hoodAngleDegrees = hoodAngleDegrees;
    }
  }
}
