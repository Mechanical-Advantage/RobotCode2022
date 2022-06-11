// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.TunableNumber;
import frc.robot.util.trajectory.CustomHolonomicDriveController;
import frc.robot.util.trajectory.CustomTrajectoryGenerator;
import frc.robot.util.trajectory.RotationSequence;
import frc.robot.util.trajectory.Waypoint;

public class SwerveMotionProfileCommand extends CommandBase {

  private static final Alert generatorAlert =
      new Alert("Failed to generate all swerve trajectories, check constants.",
          AlertType.ERROR);

  private static final TunableNumber driveKp =
      new TunableNumber("SwerveMotionProfileCommand/DriveKp");
  private static final TunableNumber driveKd =
      new TunableNumber("SwerveMotionProfileCommand/DriveKd");
  private static final TunableNumber thetaKp =
      new TunableNumber("SwerveMotionProfileCommand/ThetaKp");
  private static final TunableNumber thetaKd =
      new TunableNumber("SwerveMotionProfileCommand/ThetaKd");

  private final Drive drive;
  private final RobotState robotState;
  private final Timer timer = new Timer();
  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController =
      new PIDController(0.0, 0.0, 0.0);

  private final CustomTrajectoryGenerator trajectoryGenerator =
      new CustomTrajectoryGenerator();
  private final CustomHolonomicDriveController holonomicController =
      new CustomHolonomicDriveController(xController, yController,
          thetaController);

  /**
   * Creates a new SwerveMotionProfileCommand with no extra constraints. Drives along the specified
   * path based on odometry data.
   */
  public SwerveMotionProfileCommand(Drive drive, RobotState robotState,
      List<Waypoint> waypoints) {
    this(drive, robotState, waypoints, List.of());
  }

  /**
   * Creates a new SwerveMotionProfileCommand with extra constraints. Drives along the specified
   * path based on odometry data.
   */
  public SwerveMotionProfileCommand(Drive drive, RobotState robotState,
      List<Waypoint> waypoints, List<TrajectoryConstraint> constraints) {
    addRequirements(drive);
    this.drive = drive;
    this.robotState = robotState;

    // Select max velocity & acceleration
    boolean supportedRobot = true;
    double maxVelocityMetersPerSec, maxAccelerationMetersPerSec2,
        maxCentripetalAccelerationMetersPerSec2;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(150.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(75.0);
        driveKp.setDefault(1.0);
        driveKd.setDefault(0.0);
        thetaKp.setDefault(1.0);
        thetaKd.setDefault(0.0);
        break;
      default:
        supportedRobot = false;
        maxVelocityMetersPerSec = 0.0;
        maxAccelerationMetersPerSec2 = 0.0;
        maxCentripetalAccelerationMetersPerSec2 = 0.0;
        driveKp.setDefault(0.0);
        driveKd.setDefault(0.0);
        thetaKp.setDefault(0.0);
        thetaKd.setDefault(0.0);
        break;
    }

    // Set up trajectory configuration
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(0.25, 0.25), new Translation2d(0.25, -0.25),
        new Translation2d(-0.25, 0.25), new Translation2d(-0.25, -0.25));
    CentripetalAccelerationConstraint centripetalAccelerationConstraint =
        new CentripetalAccelerationConstraint(
            maxCentripetalAccelerationMetersPerSec2);
    TrajectoryConfig config = new TrajectoryConfig(maxVelocityMetersPerSec,
        maxAccelerationMetersPerSec2).setKinematics(kinematics)
            .addConstraint(centripetalAccelerationConstraint)
            .addConstraints(constraints);

    // Generate trajectory
    try {
      trajectoryGenerator.generate(config, waypoints);
    } catch (TrajectoryGenerationException exception) {
      if (supportedRobot) {
        generatorAlert.set(true);
        DriverStation.reportError("Failed to generate trajectory.", true);
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    Pose2d currentPose = robotState.getLatestPose();

    // Get drive setpoints
    Trajectory.State driveState =
        trajectoryGenerator.getDriveTrajectory().sample(time);
    RotationSequence.State holonomicRotationState =
        trajectoryGenerator.getHolonomicRotationSequence().sample(time);

    // Log setpoint
    Logger.getInstance().recordOutput("Odometry/ProfileSetpoint",
        new double[] {driveState.poseMeters.getX(),
            driveState.poseMeters.getY(),
            holonomicRotationState.position.getRadians()});
    Logger.getInstance().recordOutput("Odometry/ProfileDriveSetpoint",
        new double[] {driveState.poseMeters.getX(),
            driveState.poseMeters.getY(),
            driveState.poseMeters.getRotation().getRadians()});

    // Calculate speeds
    holonomicController.calculate(currentPose, driveState,
        holonomicRotationState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(
        trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }

  public double getDuration() {
    return trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds();
  }
}
