// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

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
}
