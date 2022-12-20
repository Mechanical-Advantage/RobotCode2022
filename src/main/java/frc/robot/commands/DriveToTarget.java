// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.commands.DriveWithJoysticks.AxisProcessor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;
import frc.robot.util.TunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToTarget extends CommandBase {
  private final Drive drive;
  private final RobotState robotState;
  private final Vision vision;
  private final Supplier<Double> speedSupplier;

  private static final TunableNumber convergenceFactor =
      new TunableNumber("DriveToTarget/ConvergenceFactor");
  private static final TunableNumber maxAngularSpeed =
      new TunableNumber("DriveToTarget/MaxAngularSpeed");
  private static final TunableNumber kP = new TunableNumber("DriveToTarget/kP");
  private static final TunableNumber kD = new TunableNumber("DriveToTarget/kD");

  private final AxisProcessor axisProcessor = new AxisProcessor();
  private final PIDController angularController = new PIDController(0.0, 0.0, 0.0);
  private Rotation2d closestRotation = new Rotation2d();

  private static final Pose2d[] fenderPoses =
      new Pose2d[] {
        FieldConstants.fenderA,
        FieldConstants.fenderB,
        FieldConstants.fenderAOpposite,
        FieldConstants.fenderBOpposite
      };

  /** Creates a new DriveToTarget. Guides the driver to a fender using odometry data. */
  public DriveToTarget(
      Drive drive, RobotState robotState, Vision vision, Supplier<Double> speedSupplier) {
    addRequirements(drive, vision);
    this.drive = drive;
    this.robotState = robotState;
    this.vision = vision;
    this.speedSupplier = speedSupplier;
    angularController.enableContinuousInput(-180.0, 180.0);

    convergenceFactor.setDefault(0.75);
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
        maxAngularSpeed.setDefault(0.5);
        kP.setDefault(0.01);
        kD.setDefault(0.0);
        break;
      case ROBOT_SIMBOT:
        maxAngularSpeed.setDefault(0.25);
        kP.setDefault(0.1);
        kD.setDefault(0.0);
        break;
      default:
        maxAngularSpeed.setDefault(0.0);
        kP.setDefault(0.0);
        kD.setDefault(0.0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    axisProcessor.reset(speedSupplier.get());
    angularController.reset();
    vision.setForceLeds(true);

    // Find closest rotation
    double closestDistance = Double.POSITIVE_INFINITY;
    for (Pose2d i : fenderPoses) {
      double distance = i.getTranslation().getDistance(robotState.getLatestPose().getTranslation());
      if (distance < closestDistance) {
        closestRotation = i.getRotation();
        closestDistance = distance;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Find target rotation w/ intermediate point
    double linearSpeed = axisProcessor.process(speedSupplier.get());
    Translation2d intermediatePoint = getIntermediatePoint(robotState.getLatestPose());
    Rotation2d targetRotation =
        GeomUtil.direction(intermediatePoint.minus(robotState.getLatestPose().getTranslation()));

    // Update PID gains
    if (kP.hasChanged()) {
      angularController.setP(kP.get());
    }
    if (kD.hasChanged()) {
      angularController.setP(kD.get());
    }

    // Run angular controller
    angularController.setSetpoint(targetRotation.getDegrees());
    double angularSpeed =
        angularController.calculate(
            robotState.getLatestRotation().plus(Rotation2d.fromDegrees(180.0)).getDegrees());
    angularSpeed = MathUtil.clamp(angularSpeed, -maxAngularSpeed.get(), maxAngularSpeed.get());
    drive.drivePercent(linearSpeed - angularSpeed, linearSpeed + angularSpeed);

    // Log data
    Logger.getInstance().recordOutput("ActiveCommands/DriveToTarget", true);
    Logger.getInstance()
        .recordOutput(
            "Odometry/DriveToTargetPoint",
            new double[] {
              intermediatePoint.getX(), intermediatePoint.getY(), closestRotation.getRadians()
            });
  }

  private Translation2d getIntermediatePoint(Pose2d robotPose) {
    double centerDistance = FieldConstants.hubCenter.getDistance(robotPose.getTranslation());
    Pose2d intermediatePose =
        new Pose2d(FieldConstants.hubCenter, closestRotation)
            .transformBy(
                GeomUtil.transformFromTranslation(centerDistance * convergenceFactor.get(), 0.0));
    return intermediatePose.getTranslation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    vision.setForceLeds(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
