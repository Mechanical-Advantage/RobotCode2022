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
  public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
  public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
  public static final double hangarLength = Units.inchesToMeters(128.75);
  public static final double hangarWidth = Units.inchesToMeters(116.0);

  // Vision target
  public static final double visionTargetDiameter =
      Units.inchesToMeters(4.0 * 12.0 + 5.375);
  public static final double visionTargetHeightLower =
      Units.inchesToMeters(8.0 * 12.0 + 5.625); // Bottom of tape
  public static final double visionTargetHeightUpper =
      visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape

  // Dimensions of hub and tarmac
  public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
  public static final Translation2d hubCenter =
      new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
  public static final double tarmacInnerDiameter = Units.inchesToMeters(219.25);
  public static final double tarmacOuterDiameter = Units.inchesToMeters(237.31);
  public static final double tarmacFenderToTip = Units.inchesToMeters(84.75);
  public static final double tarmacFullSideLength =
      tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
  public static final double tarmacMarkedSideLength =
      Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
  public static final double tarmacMissingSideLength =
      tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff
  public static final double hubSquareLength =
      tarmacOuterDiameter - (tarmacFenderToTip * 2.0);

  // Reference rotations (angle from hub to each reference point and fender side)
  public static final Rotation2d referenceARotation =
      Rotation2d.fromDegrees(180.0).minus(centerLineAngle)
          .plus(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d referenceBRotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceCRotation =
      referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceDRotation =
      referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d fenderARotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d fenderBRotation =
      fenderARotation.rotateBy(Rotation2d.fromDegrees(90.0));

  // Reference points (centered of the sides of the tarmac if they formed a complete octagon, plus
  // edges of fender)
  public static final Pose2d referenceA =
      new Pose2d(hubCenter, referenceARotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d referenceB =
      new Pose2d(hubCenter, referenceBRotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d referenceC =
      new Pose2d(hubCenter, referenceCRotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d referenceD =
      new Pose2d(hubCenter, referenceDRotation).transformBy(
          GeomUtil.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d fenderA =
      new Pose2d(hubCenter, fenderARotation).transformBy(
          GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));
  public static final Pose2d fenderB =
      new Pose2d(hubCenter, fenderBRotation).transformBy(
          GeomUtil.transformFromTranslation(hubSquareLength / 2.0, 0.0));

  // Cargo points
  public static final double cornerToCargoY = Units.inchesToMeters(15.56);
  public static final double referenceToCargoY =
      (tarmacFullSideLength / 2.0) - cornerToCargoY;
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

  // Terminal cargo point
  public static final Rotation2d terminalOuterRotation =
      Rotation2d.fromDegrees(133.75);
  public static final double terminalLength =
      Units.inchesToMeters(324.0 - 256.42);
  public static final double terminalWidth = Math.tan(
      Rotation2d.fromDegrees(180.0).minus(terminalOuterRotation).getRadians())
      * terminalLength;
  public static final Pose2d terminalCenter =
      new Pose2d(new Translation2d(terminalLength / 2.0, terminalWidth / 2.0),
          terminalOuterRotation.minus(Rotation2d.fromDegrees(90.0)));
  public static final double terminalCargoOffset = Units.inchesToMeters(10.43);
  public static final Pose2d cargoG = terminalCenter
      .transformBy(GeomUtil.transformFromTranslation(terminalCargoOffset, 0.0));

  // Mystery cargo
  public static final Pose2d cargoMystery = cargoB.transformBy(
      GeomUtil.transformFromTranslation(Units.inchesToMeters(24.0), 0.0));

  // Opposite reference points
  public static final Pose2d referenceAOpposite = opposite(referenceA);
  public static final Pose2d referenceBOpposite = opposite(referenceB);
  public static final Pose2d referenceCOpposite = opposite(referenceC);
  public static final Pose2d referenceDOpposite = opposite(referenceD);
  public static final Pose2d fenderAOpposite = opposite(fenderA);
  public static final Pose2d fenderBOpposite = opposite(fenderB);

  // Opposite cargo points
  public static final Pose2d cargoAOpposite = opposite(cargoA);
  public static final Pose2d cargoBOpposite = opposite(cargoB);
  public static final Pose2d cargoCOpposite = opposite(cargoC);
  public static final Pose2d cargoDOpposite = opposite(cargoD);
  public static final Pose2d cargoEOpposite = opposite(cargoE);
  public static final Pose2d cargoFOpposite = opposite(cargoF);
  public static final Pose2d cargoGOpposite = opposite(cargoG);

  // Calculate pose mirror on the opposite side of the field
  private static Pose2d opposite(Pose2d pose) {
    return new Pose2d(fieldLength, fieldWidth, Rotation2d.fromDegrees(180.0))
        .transformBy(GeomUtil.poseToTransform(pose));
  }
}
