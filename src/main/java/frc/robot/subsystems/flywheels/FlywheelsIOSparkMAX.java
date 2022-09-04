// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;
import frc.robot.util.SparkMaxDerivedVelocityController;

public class FlywheelsIOSparkMAX implements FlywheelsIO {
  private boolean invert = false;
  private double afterEncoderReduction = 1.0;

  private CANSparkMax motor;
  private RelativeEncoder defaultEncoder;
  private SparkMaxDerivedVelocityController derivedVelocityController;

  public FlywheelsIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        motor = new CANSparkMax(2, MotorType.kBrushless);
        invert = true;
        afterEncoderReduction = (48.0 / 24.0);
        break;
      default:
        throw new RuntimeException("Invalid robot for FlywheelsIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(50);
    motor.enableVoltageCompensation(12.0);

    defaultEncoder = motor.getEncoder();
    derivedVelocityController = new SparkMaxDerivedVelocityController(motor);

    motor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    Logger.getInstance().recordOutput("FlywheelTesting/DefaultRPM",
        defaultEncoder.getVelocity());
    Logger.getInstance().recordOutput("FlywheelTesting/DerivedRPM",
        derivedVelocityController.getVelocity());

    inputs.positionRad =
        Units.rotationsToRadians(derivedVelocityController.getPosition())
            / afterEncoderReduction;
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        derivedVelocityController.getVelocity()) / afterEncoderReduction;
    inputs.appliedVolts =
        motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    derivedVelocityController.disable();
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    derivedVelocityController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
            * afterEncoderReduction,
        ffVolts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    derivedVelocityController.setPID(kP, kI, kD);
  }
}
