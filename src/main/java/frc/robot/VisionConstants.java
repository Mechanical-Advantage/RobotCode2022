// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.util.TunableNumber;

/** Constants for the vision camera. */
public final class VisionConstants {
  public static final int widthPixels = 960;
  public static final int heightPixels = 720;
  public static final Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
  public static final Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);

  private static final double lowerCameraHeight = Units.inchesToMeters(37.2);
  private static final TunableNumber lowerVerticalRotationDegrees =
      new TunableNumber("VisionConstants/LowerVerticalRotationDegrees");
  private static final double lowerOffsetX = Units.inchesToMeters(12.0);

  private static final double upperCameraHeight = Units.inchesToMeters(42.5);
  private static final TunableNumber upperVerticalRotationDegrees =
      new TunableNumber("VisionConstants/UpperVerticalRotationDegrees");
  private static final double upperOffsetX = Units.inchesToMeters(9.0);

  static {
    lowerVerticalRotationDegrees.setDefault(43.3);
    upperVerticalRotationDegrees.setDefault(25.0);
  }

  public static CameraPosition getCameraPosition(HoodState hoodState) {
    switch (hoodState) {
      case LOWER:
        return new CameraPosition(lowerCameraHeight,
            Rotation2d.fromDegrees(lowerVerticalRotationDegrees.get()),
            new Transform2d(new Translation2d(lowerOffsetX, 0.0),
                Rotation2d.fromDegrees(180.0)));
      case RAISED:
        return new CameraPosition(upperCameraHeight,
            Rotation2d.fromDegrees(upperVerticalRotationDegrees.get()),
            new Transform2d(new Translation2d(upperOffsetX, 0.0),
                Rotation2d.fromDegrees(180.0)));
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
