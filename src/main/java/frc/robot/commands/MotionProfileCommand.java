// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
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
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class MotionProfileCommand extends CommandBase {
  private static final double ramseteB = 2;
  private static final double ramseteZeta = 0.7;

  private static final Alert generatorAlert =
      new Alert("Failed to generate all trajectories, check constants.", AlertType.ERROR);

  private final Drive drive;
  private final RobotState robotState;
  private final DifferentialDriveKinematics kinematics;
  private final Trajectory trajectory;
  private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
  private final Timer timer = new Timer();

  /**
   * Creates a new MotionProfileCommand with no extra constraints. Drives along the specified path
   * based on odometry data.
   */
  public MotionProfileCommand(
      Drive drive,
      RobotState robotState,
      double startVelocityMetersPerSec,
      List<Pose2d> waypoints,
      double endVelocityMetersPerSec,
      boolean reversed) {
    this(
        drive,
        robotState,
        startVelocityMetersPerSec,
        waypoints,
        endVelocityMetersPerSec,
        reversed,
        List.of());
  }

  /**
   * Creates a new MotionProfileCommand with extra constraints. Drives along the specified path
   * based on odometry data.
   */
  public MotionProfileCommand(
      Drive drive,
      RobotState robotState,
      double startVelocityMetersPerSec,
      List<Pose2d> waypoints,
      double endVelocityMetersPerSec,
      boolean reversed,
      List<TrajectoryConstraint> constraints) {
    addRequirements(drive);
    this.drive = drive;
    this.robotState = robotState;

    // Select max velocity & acceleration
    boolean supportedRobot = true;
    double maxVoltage,
        maxVelocityMetersPerSec,
        maxAccelerationMetersPerSec2,
        maxCentripetalAccelerationMetersPerSec2;
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        maxVoltage = 10.0;
        maxVelocityMetersPerSec = Units.inchesToMeters(210.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(150.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(75.0);
        break;
      case ROBOT_2020:
        maxVoltage = 10.0;
        maxVelocityMetersPerSec = Units.inchesToMeters(120.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);
        break;
      case ROBOT_ROMI:
        maxVoltage = 7.0;
        maxVelocityMetersPerSec = 0.5;
        maxAccelerationMetersPerSec2 = 0.5;
        maxCentripetalAccelerationMetersPerSec2 = 1.0;
        break;
      default:
        supportedRobot = false;
        maxVoltage = 10.0;
        maxVelocityMetersPerSec = 0.0;
        maxAccelerationMetersPerSec2 = 0.0;
        maxCentripetalAccelerationMetersPerSec2 = 0.0;
        break;
    }

    // Set up trajectory configuration
    kinematics = new DifferentialDriveKinematics(drive.getTrackWidthMeters());
    CentripetalAccelerationConstraint centripetalAccelerationConstraint =
        new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2);
    TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(kinematics)
            .addConstraint(centripetalAccelerationConstraint)
            .addConstraints(constraints)
            .setStartVelocity(startVelocityMetersPerSec)
            .setEndVelocity(endVelocityMetersPerSec)
            .setReversed(reversed);
    if (drive.getKa() != 0) {
      config.addConstraint(
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(drive.getKs(), drive.getKv(), drive.getKa()),
              kinematics,
              maxVoltage));
    }

    // Generate trajectory
    Trajectory generatedTrajectory;
    try {
      generatedTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    } catch (TrajectoryGenerationException exception) {
      generatedTrajectory = new Trajectory();
      if (supportedRobot) {
        generatorAlert.set(true);
        DriverStation.reportError("Failed to generate trajectory.", true);
      }
    }
    trajectory = generatedTrajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State setpoint = trajectory.sample(timer.get());
    Logger.getInstance()
        .recordOutput(
            "Odometry/ProfileSetpoint",
            new double[] {
              setpoint.poseMeters.getX(),
              setpoint.poseMeters.getY(),
              setpoint.poseMeters.getRotation().getRadians()
            });
    ChassisSpeeds chassisSpeeds = controller.calculate(robotState.getLatestPose(), setpoint);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    drive.driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
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
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  public double getDuration() {
    return trajectory.getTotalTimeSeconds();
  }
}
