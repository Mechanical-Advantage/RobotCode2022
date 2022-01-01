// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface for all driver and operator controls (either single or dual Xbox).
 */
public interface HandheldOI {

  public default double getLeftDriveX() {
    return 0.0;
  }

  public default double getLeftDriveY() {
    return 0.0;
  }

  public default double getRightDriveX() {
    return 0.0;
  }

  public default double getRightDriveY() {
    return 0.0;
  }

  public default Trigger getAutoAimButton() {
    return new Trigger();
  }

  public default Trigger getIntakeButton() {
    return new Trigger();
  }

  public default Trigger getShootButton() {
    return new Trigger();
  }
}
