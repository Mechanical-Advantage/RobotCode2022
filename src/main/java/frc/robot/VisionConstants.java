// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.hood.Hood.HoodState;

/** Constants for the vision camera. */
public final class VisionConstants {
  public static final int widthPixels = 960;
  public static final int heightPixels = 720;
  public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
  public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);

  public static final CameraPosition lowerPosition =
      new CameraPosition(0.0, new Rotation2d(), new Transform2d());
  public static final CameraPosition raisedPosition =
      new CameraPosition(0.0, new Rotation2d(), new Transform2d());

  public static CameraPosition getCameraPosition(HoodState hoodState) {
    switch (hoodState) {
      case LOWER:
        return lowerPosition;
      case RAISED:
        return raisedPosition;
      default:
        return null;
    }
  }

  public static final class CameraPosition {
    public final double cameraHeight;
    public final Rotation2d verticalRotation;
    public final Transform2d vehicleToCamera;

    public CameraPosition(double cameraHeight, Rotation2d verticalRotation,
        Transform2d vehicleToCamera) {
      this.cameraHeight = cameraHeight;
      this.verticalRotation = verticalRotation;
      this.vehicleToCamera = vehicleToCamera;
    }
  }
}
