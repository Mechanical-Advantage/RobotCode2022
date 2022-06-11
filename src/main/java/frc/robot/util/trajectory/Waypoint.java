package frc.robot.util.trajectory;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A trajectory waypoint, including a translation and optional drive/holonomic rotations. */
public class Waypoint {
  private final Translation2d translation;
  private final Rotation2d driveRotation;
  private final Rotation2d holonomicRotation;

  /**
   * Constructs a Waypoint at the origin and without a drive or holonomic rotation.
   */
  public Waypoint() {
    this(new Translation2d());
  }

  /**
   * Constructs a Waypoint with a translation, drive rotation, and holonomic rotation.
   * 
   * @param translation Waypoint position (required)
   * @param driveRotation Drive velocity rotation (optional, can be null)
   * @param holonomicRotation Holonomic rotation (optional, can be null)
   */
  public Waypoint(Translation2d translation, Rotation2d driveRotation,
      Rotation2d holonomicRotation) {
    this.translation =
        requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = driveRotation;
    this.holonomicRotation = holonomicRotation;
  }

  /**
   * Constructs a Waypoint with a pose and holonomic rotation.
   * 
   * @param pose Waypoint position and drive velocity rotation (required)
   * @param holonomicRotation Holonomic rotation (optional, can be null)
   */
  public Waypoint(Pose2d pose, Rotation2d holonomicRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    this.translation = pose.getTranslation();
    this.driveRotation = pose.getRotation();
    this.holonomicRotation = holonomicRotation;
  }

  /**
   * Constructs a Waypoint with a translation and drive rotation (but no holonomic rotation).
   * 
   * @param translation Waypoint position (required)
   * @param driveRotation Drive velocity rotation (optional, can be null)
   */
  public Waypoint(Translation2d translation, Rotation2d driveRotation) {
    this.translation =
        requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = driveRotation;
    this.holonomicRotation = null;
  }

  /**
   * Constructs a Waypoint from a pose (rotation is used as drive rotation)
   * 
   * @param pose Waypoint position and drive rotation (required)
   */
  public Waypoint(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    this.translation = pose.getTranslation();
    this.driveRotation = pose.getRotation();
    this.holonomicRotation = null;
  }

  /**
   * Constructs a Waypoint with a translation (but no drive or holonomic rotation).
   * 
   * @param translation Waypoint position (required)
   */
  public Waypoint(Translation2d translation) {
    this.translation =
        requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = null;
    this.holonomicRotation = null;
  }

  /** Returns the translation component of the waypoint. */
  public Translation2d getTranslation() {
    return translation;
  }

  /**
   * Returns the drive rotation component of the waypoint (or an empty optional if not specified).
   */
  public Optional<Rotation2d> getDriveRotation() {
    return Optional.ofNullable(driveRotation);
  }

  /**
   * Returns the holonomic rotation component of the waypoint (or an empty optional if not
   * specified).
   */
  public Optional<Rotation2d> getHolonomicRotation() {
    return Optional.ofNullable(holonomicRotation);
  }
}
