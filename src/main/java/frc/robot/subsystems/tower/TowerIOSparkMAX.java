// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class TowerIOSparkMAX implements TowerIO {
  private boolean invert = false;
  private double afterEncoderReduction = 1.0;

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput cargoSensor;

  public TowerIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        motor = new CANSparkMax(30, MotorType.kBrushless);
        cargoSensor = new DigitalInput(0);
        invert = false;
        afterEncoderReduction = 9.0;
        break;
      default:
        throw new RuntimeException("Invalid robot for TowerIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(30);
    motor.enableVoltageCompensation(12.0);

    encoder = motor.getEncoder();

    motor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(TowerIOInputs inputs) {
    inputs.cargoSensor = cargoSensor.get();
    inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition()) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            / afterEncoderReduction;
    inputs.appliedVolts = motor.getAppliedOutput() * 12.0;
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
