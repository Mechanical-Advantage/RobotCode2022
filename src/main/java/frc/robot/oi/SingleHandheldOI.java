// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements HandheldOI {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getLeftDriveX() {
    return controller.getLeftX();
  }

  @Override
  public double getLeftDriveY() {
    return controller.getLeftY() * -1;
  }

  @Override
  public double getRightDriveX() {
    return controller.getRightX();
  }

  @Override
  public double getRightDriveY() {
    return controller.getRightY() * -1;
  }

  @Override
  public Trigger getAutoDriveButton() {
    return new Trigger(controller::getLeftBumper);
  }

  @Override
  public Trigger getAutoAimButton() {
    return new Trigger(controller::getRightBumper);
  }

  @Override
  public Trigger getShootButton() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger getLogMarkerButton() {
    return new Trigger(controller::getStartButton)
        .or(new Trigger(controller::getBackButton));
  }

  @Override
  public Trigger getIntakeForwardsButton() {
    return new Trigger(() -> controller.getRightTriggerAxis() > 0.95);
  }

  @Override
  public Trigger getIntakeBackwardsButton() {
    return new Trigger(() -> controller.getLeftTriggerAxis() > 0.95);
  }

  @Override
  public Trigger getStopFlywheelButton() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getStartFlywheelFenderButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getStartFlywheelTarmacButton() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getStartFlywheelAutoButton() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getDuckSoundButton1() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getDuckSoundButton2() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getDuckSoundButton3() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getDuckSoundButton4() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger getShooterIncrement() {
    return new Trigger(() -> controller.getPOV() == 90);
  }

  @Override
  public Trigger getShooterDecrement() {
    return new Trigger(() -> controller.getPOV() == 270);
  }

  @Override
  public Trigger getClimbTop() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getClimbBottom() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getClimbAuto() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger getClimbReset() {
    return new Trigger(() -> controller.getPOV() == 180);
  }

  @Override
  public double getClimbStick() {
    return controller.getLeftY() * -1;
  }

  @Override
  public void setDriverRumble(double percent) {
    controller.setRumble(RumbleType.kRightRumble, percent);
  }

  @Override
  public void setOperatorRumble(double percent) {
    controller.setRumble(RumbleType.kRightRumble, percent);
  }
}
