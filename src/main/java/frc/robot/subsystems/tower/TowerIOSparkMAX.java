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

  private final DigitalInput lowerCargoSensor1;
  private final DigitalInput lowerCargoSensor2;
  private final DigitalInput upperCargoSensor1;
  private final DigitalInput upperCargoSensor2;

  public TowerIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        motor = new CANSparkMax(30, MotorType.kBrushless);
        invert = false;
        afterEncoderReduction = 9.0;

        lowerCargoSensor1 = new DigitalInput(4);
        lowerCargoSensor2 = new DigitalInput(5);
        upperCargoSensor1 = new DigitalInput(6);
        upperCargoSensor2 = new DigitalInput(7);
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
    inputs.cargoSensorsAvailable = true;
    inputs.lowerCargoSensor1 = lowerCargoSensor1.get();
    inputs.lowerCargoSensor2 = lowerCargoSensor2.get();
    inputs.upperCargoSensor1 = upperCargoSensor1.get();
    inputs.upperCargoSensor2 = upperCargoSensor2.get();

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
