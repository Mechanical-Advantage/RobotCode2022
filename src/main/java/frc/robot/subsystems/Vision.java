// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.util.CircleFitter;
import frc.robot.util.GeomUtil;

public class Vision extends SubsystemBase {
  private static final Rotation2d horizontalPlaneToLens =
      Rotation2d.fromDegrees(30.0);
  private static final double lensHeightMeters = Units.inchesToMeters(18.25);
  private static final Transform2d vehicleToCamera = new Transform2d(
      new Translation2d(Units.inchesToMeters(1.875), 0.0), new Rotation2d());
  private static final double circleFitPrecision = 0.01;

  private PhotonCamera camera = new PhotonCamera("photonvision");
  private Supplier<Rotation2d> rotationSupplier = () -> new Rotation2d();

  /** Creates a new Vision. */
  public Vision() {}

  /** Sets pose supplier from drive. TODO: Process in drive subsystem. */
  public void setRotationSupplier(Supplier<Rotation2d> supplier) {
    rotationSupplier = supplier;
  }

  @Override
  public void periodic() {
    List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;

    if (targets.size() > 0) {
      List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
      for (PhotonTrackedTarget target : targets) {
        List<TargetCorner> corners = target.getCorners().stream()
            .sorted(sortByY).collect(Collectors.toList());

        cameraToTargetTranslations.add(solveCameraToTargetTranslation(
            corners.get(0), FieldConstants.visionTargetHeightUpper));
        cameraToTargetTranslations.add(solveCameraToTargetTranslation(
            corners.get(1), FieldConstants.visionTargetHeightUpper));
        cameraToTargetTranslations.add(solveCameraToTargetTranslation(
            corners.get(2), FieldConstants.visionTargetHeightLower));
        cameraToTargetTranslations.add(solveCameraToTargetTranslation(
            corners.get(3), FieldConstants.visionTargetHeightLower));
      }

      Translation2d cameraToTargetTranslation =
          CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
              cameraToTargetTranslations, circleFitPrecision);

      // --- CALCULATE NEW ROBOT POSE ---
      Rotation2d cameraRotation =
          rotationSupplier.get().rotateBy(vehicleToCamera.getRotation());
      Transform2d fieldToTargetRotated =
          new Transform2d(FieldConstants.hubCenter, cameraRotation);
      Transform2d fieldToCamera = fieldToTargetRotated.plus(GeomUtil
          .transformFromTranslation(cameraToTargetTranslation.unaryMinus()));
      Pose2d fieldToVehicle = GeomUtil
          .transformToPose(fieldToCamera.plus(vehicleToCamera.inverse()));
    }
  }

  public static void main(String... args) {
    Transform2d test =
        new Transform2d(new Translation2d(3, 1), Rotation2d.fromDegrees(80));
    System.out.println(test);
    System.out.println(test.plus(new Transform2d(new Translation2d(8, 0),
        Rotation2d.fromDegrees(20.0))));
  }

  private double vpw = 2.0 * Math.tan(Math.toRadians(59.6 / 2.0));
  private double vph = 2.0 * Math.tan(Math.toRadians(49.7 / 2.0));

  private Translation2d solveCameraToTargetTranslation(TargetCorner corner,
      double goalHeight) {
    double yPixels = corner.x;
    double zPixels = corner.y;

    // Robot frame of reference
    double nY = -((yPixels - 160.0) / 160.0);
    double nZ = -((zPixels - 120.0) / 120.0);

    Translation2d xzPlaneTranslation =
        new Translation2d(1.0, vph / 2.0 * nZ).rotateBy(horizontalPlaneToLens);
    double x = xzPlaneTranslation.getX();
    double y = vpw / 2.0 * nY;
    double z = xzPlaneTranslation.getY();

    double differentialHeight = lensHeightMeters - goalHeight;
    if ((z < 0.0) == (differentialHeight > 0.0)) {
      double scaling = differentialHeight / -z;
      double distance = Math.hypot(x, y) * scaling;
      Rotation2d angle = new Rotation2d(x, y);
      return new Translation2d(distance * angle.getCos(),
          distance * angle.getSin());
    }
    return null;
  }

  private static final Comparator<TargetCorner> sortByY = (TargetCorner c1,
      TargetCorner c2) -> Double.valueOf(c1.y).compareTo(Double.valueOf(c2.y));
}
