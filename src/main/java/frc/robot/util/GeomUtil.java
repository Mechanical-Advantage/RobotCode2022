// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
  /**
   * Creates a pure translating transform
   * 
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d transformFromTranslation(
      Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   * 
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d transformFromTranslation(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   * 
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d transformFromRotation(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Creates a pure translated pose
   * 
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d poseFromTranslation(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   * 
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d poseFromRotation(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   * 
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   * 
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Interpolates between two poses based on the scale factor t. For example, t=0 would result in
   * the first pose, t=1 would result in the last pose, and t=0.5 would result in a pose which is
   * exactly halfway between the two poses. Values of t less than zero return the first pose, and
   * values of t greater than 1 return the last pose.
   * 
   * @param lhs The left hand side, or first pose to use for interpolation
   * @param rhs The right hand side, or last pose to use for interpolation
   * @param t The scale factor, 0 <= t <= 1
   * @return The pose which represents the interpolation. For t <= 0, the "lhs" parameter is
   *         returned directly. For t >= 1, the "rhs" parameter is returned directly.
   */
  public static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {
    if (t <= 0) {
      return lhs;
    } else if (t >= 1) {
      return rhs;
    }
    Twist2d twist = lhs.log(rhs);
    Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
    return lhs.exp(scaled);
  }

  /**
   * Returns the direction that this translation makes with the origin as a Rotation2d
   * 
   * @param translation The translation
   * @return The direction of the translation
   */
  public static Rotation2d direction(Translation2d translation) {
    return new Rotation2d(translation.getX(), translation.getY());
  }
}
