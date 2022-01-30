// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualHandheldOI implements HandheldOI {
  private final XboxController driverController;
  private final XboxController operatorController;

  public DualHandheldOI(int driverPort, int operatorPort) {
    driverController = new XboxController(driverPort);
    operatorController = new XboxController(operatorPort);
  }

  @Override
  public double getLeftDriveX() {
    return driverController.getLeftX();
  }

  @Override
  public double getLeftDriveY() {
    return driverController.getLeftY() * -1;
  }

  @Override
  public double getRightDriveX() {
    return driverController.getRightX();
  }

  @Override
  public double getRightDriveY() {
    return driverController.getRightY() * -1;
  }

  @Override
  public Trigger getSniperModeButton() {
    return new Trigger(driverController::getRightBumper);
  }

  @Override
  public Trigger getAutoAimButton() {
    return new Trigger(driverController::getLeftBumper);
  }

  @Override
  public Trigger getIntakeExtendButton() {
    return new Trigger(operatorController::getRightBumper);
  }

  @Override
  public Trigger getIntakeRetractButton() {
    return new Trigger(operatorController::getLeftBumper);
  }

  @Override
  public Trigger getIntakeForwardsButton() {
    return new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5);
  }

  @Override
  public Trigger getIntakeBackwardsButton() {
    return new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5);
  }

  @Override
  public Trigger getTowerUpButton() {
    return new Trigger(() -> operatorController.getPOV() == 0);
  }

  @Override
  public Trigger getTowerDownButton() {
    return new Trigger(() -> operatorController.getPOV() == 180);
  }
}
