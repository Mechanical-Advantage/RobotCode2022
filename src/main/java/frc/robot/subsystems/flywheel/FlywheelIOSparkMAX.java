// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Template hardware implementation for a closed loop subsystem. */
public class FlywheelIOSparkMAX implements FlywheelIO {
  private boolean invert = false;
  private boolean invertFollower = false;
  private double gearRatio = 1.0;

  private CANSparkMax leader;
  private CANSparkMax follower;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;

  public FlywheelIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        leader = new CANSparkMax(0, MotorType.kBrushless);
        follower = new CANSparkMax(1, MotorType.kBrushless);
        invert = false;
        invertFollower = false;
        gearRatio = 1.0;
        break;
      default:
        throw new RuntimeException("Invalid robot for FlywheelIOSparkMax!");
    }

    if (Constants.burnMotorControllerFlash) {
      leader.restoreFactoryDefaults();
      follower.restoreFactoryDefaults();
    }

    follower.follow(leader, invertFollower);
    leader.setInverted(invert);
    leader.setSmartCurrentLimit(30);
    follower.setSmartCurrentLimit(30);
    leader.enableVoltageCompensation(12.0);
    follower.enableVoltageCompensation(12.0);

    encoder = leader.getEncoder();
    pid = leader.getPIDController();

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);

    if (Constants.burnMotorControllerFlash) {
      leader.burnFlash();
      follower.burnFlash();
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = encoder.getPosition() * gearRatio * 2 * Math.PI;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            * gearRatio;
    inputs.appliedVolts = leader.getAppliedOutput();
    inputs.currentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.tempCelcius = new double[] {leader.getMotorTemperature(),
        follower.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
            / gearRatio,
        ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configurePID(double kp, double ki, double kd) {
    pid.setP(kp, 0);
    pid.setI(ki, 0);
    pid.setD(kd, 0);
    pid.setFF(0, 0);
  }
}
