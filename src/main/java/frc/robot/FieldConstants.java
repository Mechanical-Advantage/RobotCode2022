// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.GeomUtil;

/** Contains various field dimensions and useful side points. All dimensions are in meters. */
public final class FieldConstants {

  // Field dimensions
  public static final double fieldLength = Units.inchesToMeters(54.0 * 12);
  public static final double fieldWidth = Units.inchesToMeters(27.0 * 12);
  public static final double hangarLength = Units.inchesToMeters(128.75);
  public static final double hangarWidth = Units.inchesToMeters(116.0);

  // Dimensions of hub and tarmac
  public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66);
  public static final Translation2d hubCenter =
      new Translation2d(fieldLength / 2, fieldWidth / 2);
  public static final double tarmacHeight = Units.inchesToMeters(219.25); // Between parallel sides
  public static final double tarmacFullSideLength =
      tarmacHeight * (Math.sqrt(2) - 1); // If the tarmac formed a full octagon
  public static final double tarmacMarkedSideLength =
      Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
  public static final double tarmacMissingSideLength =
      tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff

  // Reference rotations (angle from hub to each reference point)
  public static final Rotation2d referenceARotation =
      Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
          .plus(Rotation2d.fromDegrees(360.0 / 16));
  public static final Rotation2d referenceBRotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8));
  public static final Rotation2d referenceCRotation =
      referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8));
  public static final Rotation2d referenceDRotation =
      referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8));

  // Reference points (centered of the sides of the tarmac if they formed a complete octagon)
  public static final Pose2d referenceA =
      new Pose2d(hubCenter, referenceARotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacHeight / 2, 0.0));
  public static final Pose2d referenceB =
      new Pose2d(hubCenter, referenceBRotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacHeight / 2, 0.0));
  public static final Pose2d referenceC =
      new Pose2d(hubCenter, referenceCRotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacHeight / 2, 0.0));
  public static final Pose2d referenceD =
      new Pose2d(hubCenter, referenceDRotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacHeight / 2, 0.0));

  // Cargo points
  public static final double cornerToCargoY = Units.inchesToMeters(15.56);
  public static final double referenceToCargoY =
      (tarmacFullSideLength / 2) - cornerToCargoY;
  public static final double referenceToCargoX = Units.inchesToMeters(40.44);
  public static final Pose2d cargoA = referenceA.transformBy(
      GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoB = referenceA.transformBy(
      GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
  public static final Pose2d cargoC = referenceB.transformBy(
      GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
  public static final Pose2d cargoD = referenceC.transformBy(
      GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoE = referenceD.transformBy(
      GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoF = referenceD.transformBy(
      GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
}
