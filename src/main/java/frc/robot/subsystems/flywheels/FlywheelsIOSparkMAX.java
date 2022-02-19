// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class FlywheelsIOSparkMAX implements FlywheelsIO {
  private boolean bigInvert = false;
  private double bigAfterEncoderReduction = 1.0;

  private boolean littleInvert = false;
  private double littleAfterEncoderReduction = 1.0;

  private CANSparkMax bigMotor;
  private RelativeEncoder bigEncoder;
  private SparkMaxPIDController bigPID;

  private CANSparkMax littleMotor;
  private RelativeEncoder littleEncoder;
  private SparkMaxPIDController littlePID;

  public FlywheelsIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        bigMotor = new CANSparkMax(2, MotorType.kBrushless);
        littleMotor = new CANSparkMax(3, MotorType.kBrushless);
        bigInvert = true;
        bigAfterEncoderReduction = (48.0 / 24.0);
        littleInvert = false;
        littleAfterEncoderReduction = (1.0 / 2.0);
        break;
      default:
        throw new RuntimeException("Invalid robot for FlywheelsIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      bigMotor.restoreFactoryDefaults();
      littleMotor.restoreFactoryDefaults();
    }

    bigMotor.setInverted(bigInvert);
    littleMotor.setInverted(littleInvert);
    bigMotor.setSmartCurrentLimit(50);
    littleMotor.setSmartCurrentLimit(40);
    bigMotor.enableVoltageCompensation(12.0);
    littleMotor.enableVoltageCompensation(12.0);

    bigEncoder = bigMotor.getEncoder();
    littleEncoder = littleMotor.getEncoder();
    bigPID = bigMotor.getPIDController();
    littlePID = littleMotor.getPIDController();

    bigMotor.setCANTimeout(0);
    littleMotor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      bigMotor.burnFlash();
      littleMotor.burnFlash();
    }
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.bigPositionRad = Units.rotationsToRadians(bigEncoder.getPosition())
        / bigAfterEncoderReduction;
    inputs.bigVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bigEncoder.getVelocity())
            / bigAfterEncoderReduction;
    inputs.bigAppliedVolts = bigMotor.getAppliedOutput() * 12.0;
    inputs.bigCurrentAmps = new double[] {bigMotor.getOutputCurrent(),};
    inputs.bigTempCelcius = new double[] {bigMotor.getMotorTemperature(),};

    inputs.littlePositionRad =
        Units.rotationsToRadians(littleEncoder.getPosition())
            / littleAfterEncoderReduction;
    inputs.littleVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(littleEncoder.getVelocity())
            / littleAfterEncoderReduction;
    inputs.littleAppliedVolts = littleMotor.getAppliedOutput() * 12.0;
    inputs.littleCurrentAmps = new double[] {littleMotor.getOutputCurrent()};
    inputs.littleTempCelcius = new double[] {littleMotor.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double bigVolts, double littleVolts) {
    bigMotor.setVoltage(bigVolts);
    littleMotor.setVoltage(littleVolts);
  }

  @Override
  public void setVelocity(double bigVelocityRadPerSec,
      double littleVelocityRadPerSec, double bigFFVolts, double littleFFVolts) {
    bigPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(bigVelocityRadPerSec)
            * bigAfterEncoderReduction,
        ControlType.kVelocity, 0, bigFFVolts, ArbFFUnits.kVoltage);
    littlePID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(littleVelocityRadPerSec)
            * littleAfterEncoderReduction,
        ControlType.kVelocity, 0, littleFFVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    bigMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    littleMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configurePID(double bigKp, double bigKi, double bigKd,
      double littleKp, double littleKi, double littleKd) {
    bigPID.setP(bigKp, 0);
    bigPID.setI(bigKi, 0);
    bigPID.setD(bigKd, 0);
    bigPID.setFF(0, 0);

    littlePID.setP(littleKp, 0);
    littlePID.setI(littleKi, 0);
    littlePID.setD(littleKd, 0);
    littlePID.setFF(0, 0);
  }
}
