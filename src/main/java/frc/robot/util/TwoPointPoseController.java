// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Guides the robot to a target pose using an intermediate point to converge.
 */
public class TwoPointPoseController {

  private final static double defaultConvergenceFactor = 0.5; // How quickly to converge behind the
                                                              // target
  private final static double defaultMaxLinearAngleRadians = 0.25 * Math.PI; // Max angle error at
                                                                             // which linear
                                                                             // velocity
                                                                             // is allowed

  private Pose2d target = new Pose2d();
  private boolean linearAtSetpoint = false;
  private final double convergenceFactor;
  private final double maxLinearAngleRadians;
  private final PIDController linearController;
  private final Translation2d linearTolerance;
  private final PIDController angularController;

  /**
   * Creates a new TwoPointPoseController with default convergence factor and max linear angle.
   * 
   * @param linearController PID controller for correcting distance to target
   * @param linearToleranceMeters Acceptable target distance in meters (x and y)
   * @param angularController PID controller for correcting angle to target
   * @param angularToleranceRadians Acceptable angular error in radians
   */
  public TwoPointPoseController(PIDController linearController,
      Translation2d linearToleranceMeters, PIDController angularController,
      double angularToleranceRadians) {
    this(defaultConvergenceFactor, defaultMaxLinearAngleRadians,
        linearController, linearToleranceMeters, angularController,
        angularToleranceRadians);
  }

  /**
   * Creates a new TwoPointPoseController with custom convergence factor and max linear angle.
   * 
   * @param convergenceFactor (0-1) How quickly to converge behind the target, lower values correct
   *        heading closer to the target
   * @param maxLinearAngleRadians Maximum allowable error on the angular controller where linear
   *        velocity is used, allows for turning in place
   * @param linearController PID controller for correcting distance to target
   * @param linearToleranceMeters Acceptable target distance in meters (x and y)
   * @param angularController PID controller for correcting angle to target
   * @param angularToleranceRadians Acceptable angular error in radians
   */
  public TwoPointPoseController(double convergenceFactor,
      double maxLinearAngleRadians, PIDController linearController,
      Translation2d linearToleranceMeters, PIDController angularController,
      double angularToleranceRadians) {
    this.convergenceFactor = convergenceFactor;
    this.maxLinearAngleRadians = maxLinearAngleRadians;
    this.linearController = linearController;
    this.linearTolerance = linearToleranceMeters;
    linearController.setSetpoint(0);
    linearController.disableContinuousInput();
    this.angularController = angularController;
    angularController.setTolerance(angularToleranceRadians);
    angularController.enableContinuousInput(Math.PI * -1, Math.PI);
  }

  /**
   * Updates the target pose
   * 
   * @param target The required final pose
   */
  public void setTarget(Pose2d target) {
    this.target = target;
  }

  /**
   * Resets the angular and linear PID controllers/
   */
  public void reset() {
    linearController.reset();
    angularController.reset();
    linearAtSetpoint = false;
  }

  /**
   * Calculates next set of output speeds.
   * 
   * @param currentPose The robot's current position from odometry
   * @return Calculated chassis speeds
   */
  public ChassisSpeeds calculate(Pose2d currentPose) {
    ChassisSpeeds output = new ChassisSpeeds();

    // Determine intermediate target
    double xDist = new Transform2d(target, currentPose).getX();
    Translation2d intermediateTarget = target
        .transformBy(new Transform2d(
            new Translation2d(xDist * convergenceFactor, 0), new Rotation2d()))
        .getTranslation();

    // Calculate relative target positions
    Translation2d relativeIntermediateTarget =
        intermediateTarget.minus(currentPose.getTranslation());
    Pose2d relativeTarget = target.relativeTo(currentPose);

    // Run PID controllers
    output.vxMetersPerSecond = linearController.calculate(
        relativeTarget.getTranslation().getNorm() * (xDist > 0 ? 1 : -1));
    linearAtSetpoint = Math.abs(relativeTarget.getX()) < linearTolerance.getX()
        && Math.abs(relativeTarget.getY()) < linearTolerance.getY();
    if (linearAtSetpoint) { // We're at the target, point in final direction
      output.vxMetersPerSecond = 0;
      angularController.setSetpoint(target.getRotation().getRadians());
    } else { // Not at the target yet, point at the intermediate target
      angularController.setSetpoint(new Rotation2d(
          relativeIntermediateTarget.getX(), relativeIntermediateTarget.getY())
              .rotateBy(new Rotation2d(xDist > 0 ? Math.PI : 0)).getRadians());
    }
    output.omegaRadiansPerSecond =
        angularController.calculate(currentPose.getRotation().getRadians());
    if (angularController.atSetpoint()) {
      output.omegaRadiansPerSecond = 0;
    }

    // Decrease linear output when not facing correct direction
    double linearPower = 1 - Math
        .abs(angularController.getPositionError() / maxLinearAngleRadians);
    linearPower = linearPower < 0 ? 0 : linearPower;
    output.vxMetersPerSecond *= linearPower;

    return output;
  }

  /**
   * Checks whether the robot is at the target (distance and heading)
   */
  public boolean atTarget() {
    return linearAtSetpoint && angularController.atSetpoint();
  }
}
