// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.TreeMap;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import frc.robot.util.GeomUtil;
import frc.robot.util.TunableNumber;
import frc.robot.util.Alert.AlertType;

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

  private Trajectory driveTrajectory;
  private final TreeMap<Double, Rotation2d> thetaPoints = new TreeMap<>();

  /**
   * Creates a new SwerveMotionProfileCommand with no extra constraints. Drives along the specified
   * path based on odometry data.
   */
  public SwerveMotionProfileCommand(Drive drive, RobotState robotState,
      List<Pose2d> waypoints) {
    this(drive, robotState, waypoints, List.of());
  }

  /**
   * Creates a new SwerveMotionProfileCommand with extra constraints. Drives along the specified
   * path based on odometry data.
   */
  public SwerveMotionProfileCommand(Drive drive, RobotState robotState,
      List<Pose2d> waypoints, List<TrajectoryConstraint> constraints) {
    addRequirements(drive);
    this.drive = drive;
    this.robotState = robotState;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

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

    // Generate drive trajectory
    if (waypoints.size() >= 2) {
      Pose2d start = new Pose2d(waypoints.get(0).getTranslation(),
          GeomUtil.direction(waypoints.get(1).getTranslation()
              .minus(waypoints.get(0).getTranslation())));
      int lastIndex = waypoints.size() - 1;
      Pose2d end = new Pose2d(waypoints.get(lastIndex).getTranslation(),
          GeomUtil.direction(waypoints.get(lastIndex).getTranslation()
              .minus(waypoints.get(lastIndex - 1).getTranslation())));
      List<Translation2d> interiorWaypoints =
          waypoints.subList(1, lastIndex).stream()
              .map(pose -> pose.getTranslation()).collect(Collectors.toList());

      try {
        driveTrajectory = TrajectoryGenerator.generateTrajectory(start,
            interiorWaypoints, end, config);
      } catch (TrajectoryGenerationException exception) {
      }
    }

    // Check for generation error
    if (driveTrajectory == null) {
      driveTrajectory = new Trajectory();
      if (supportedRobot) {
        generatorAlert.set(true);
        DriverStation.reportError("Failed to generate trajectory.", true);
      }
    }

    // Find theta points
    thetaPoints.put(0.0, waypoints.get(0).getRotation());
    thetaPoints.put(driveTrajectory.getTotalTimeSeconds(),
        waypoints.get(waypoints.size() - 1).getRotation());
    if (waypoints.size() > 2) {
      int index = 1;
      for (Trajectory.State state : driveTrajectory.getStates()) {
        if (state.poseMeters.getTranslation()
            .getDistance(waypoints.get(index).getTranslation()) < 1E-3) {
          thetaPoints.put(state.timeSeconds,
              waypoints.get(index).getRotation());
          index += 1;
          if (index >= waypoints.size() - 1) {
            break;
          }
        }
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

    // Get drive setpoint
    Trajectory.State driveState = driveTrajectory.sample(time);

    // Get theta setpoint
    double thetaSetpointRadians;
    double thetaVelocityRadiansPerSec;

    Entry<Double, Rotation2d> lastThetaPoint = thetaPoints.floorEntry(time);
    Entry<Double, Rotation2d> nextThetaPoint = thetaPoints.ceilingEntry(time);
    if (nextThetaPoint == null) {
      thetaSetpointRadians = lastThetaPoint.getValue().getRadians();
      thetaVelocityRadiansPerSec = 0.0;
    } else {
      double thetaAccelerationRadiansPerSec2 = (4 * nextThetaPoint.getValue()
          .minus(lastThetaPoint.getValue()).getRadians())
          / Math.pow(nextThetaPoint.getKey() - lastThetaPoint.getKey(), 2);

      if (time < (nextThetaPoint.getKey() + lastThetaPoint.getKey()) / 2) { // Accelerating
        thetaSetpointRadians = lastThetaPoint.getValue().getRadians()
            + ((thetaAccelerationRadiansPerSec2 / 2)
                * Math.pow(time - lastThetaPoint.getKey(), 2));
        thetaVelocityRadiansPerSec =
            (time - lastThetaPoint.getKey()) * thetaAccelerationRadiansPerSec2;

      } else { // Decelerating
        thetaSetpointRadians = nextThetaPoint.getValue().getRadians()
            - ((thetaAccelerationRadiansPerSec2 / 2)
                * Math.pow(time - nextThetaPoint.getKey(), 2));
        thetaVelocityRadiansPerSec =
            (nextThetaPoint.getKey() - time) * thetaAccelerationRadiansPerSec2;
      }
      while (thetaSetpointRadians > Math.PI) {
        thetaSetpointRadians -= Math.PI * 2;
      }
      while (thetaSetpointRadians < -Math.PI) {
        thetaSetpointRadians += Math.PI * 2;
      }
    }

    // Log setpoint
    Logger.getInstance().recordOutput("Odometry/ProfileSetpoint",
        new double[] {driveState.poseMeters.getX(),
            driveState.poseMeters.getY(), thetaSetpointRadians});
    Logger.getInstance().recordOutput("Odometry/ProfileDriveSetpoint",
        new double[] {driveState.poseMeters.getX(),
            driveState.poseMeters.getY(),
            driveState.poseMeters.getRotation().getRadians()});

    // Calculate feedforward velocities
    double xFeedforward = driveState.velocityMetersPerSecond
        * driveState.poseMeters.getRotation().getCos();
    double yFeedforward = driveState.velocityMetersPerSecond
        * driveState.poseMeters.getRotation().getSin();
    double thetaFeedforward = thetaVelocityRadiansPerSec;

    // Calculate feedback velocities
    double xFeedback =
        xController.calculate(currentPose.getX(), driveState.poseMeters.getX());
    double yFeedback =
        yController.calculate(currentPose.getY(), driveState.poseMeters.getY());
    double thetaFeedback = thetaController.calculate(
        currentPose.getRotation().getRadians(), thetaSetpointRadians);

    // Drive at calculated speeds
    ChassisSpeeds.fromFieldRelativeSpeeds(xFeedforward + xFeedback,
        yFeedforward + yFeedback, thetaFeedforward + thetaFeedback,
        currentPose.getRotation());
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
    return timer.hasElapsed(driveTrajectory.getTotalTimeSeconds());
  }

  public double getDuration() {
    return driveTrajectory.getTotalTimeSeconds();
  }
}
