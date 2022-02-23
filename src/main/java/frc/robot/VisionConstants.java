// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.Hood.HoodState;

/** Constants for the vision camera. */
public final class VisionConstants {
  public static final String cameraName = "limelight";
  public static final int widthPixels = 960;
  public static final int heightPixels = 720;
  public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
  public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);

  public static final CameraPosition lowerPosition = new CameraPosition(
      Units.inchesToMeters(36.642), Rotation2d.fromDegrees(67.454),
      new Transform2d(new Translation2d(Units.inchesToMeters(11.902), 0.0),
          Rotation2d.fromDegrees(180.0)));
  public static final CameraPosition raisedPosition = new CameraPosition(
      Units.inchesToMeters(42.588), Rotation2d.fromDegrees(43.29),
      new Transform2d(new Translation2d(Units.inchesToMeters(8.875), 0.0),
          Rotation2d.fromDegrees(180.0)));

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
