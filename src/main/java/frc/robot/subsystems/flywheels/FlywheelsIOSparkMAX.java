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

public class FlywheelsIOSparkMAX implements FlywheelsIO {
  private boolean bigInvert = false;
  private boolean bigInvertFollower = false;
  private double bigAfterEncoderReduction = 1.0;

  private boolean littleInvert = false;
  private boolean littleInvertFollower = false;
  private double littleAfterEncoderReduction = 1.0;

  private CANSparkMax bigLeader;
  private CANSparkMax bigFollower;
  private RelativeEncoder bigEncoder;
  private SparkMaxPIDController bigPID;

  private CANSparkMax littleLeader;
  private CANSparkMax littleFollower;
  private RelativeEncoder littleEncoder;
  private SparkMaxPIDController littlePID;

  public FlywheelsIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        bigLeader = new CANSparkMax(0, MotorType.kBrushless);
        bigFollower = new CANSparkMax(1, MotorType.kBrushless);
        littleLeader = new CANSparkMax(2, MotorType.kBrushless);
        littleFollower = new CANSparkMax(3, MotorType.kBrushless);
        bigInvert = false;
        bigInvertFollower = false;
        bigAfterEncoderReduction = 1.0;
        littleInvertFollower = false;
        littleAfterEncoderReduction = 1.0;

        break;
      default:
        throw new RuntimeException("Invalid robot for FlywheelsIOSparkMax!");
    }

    if (Constants.burnMotorControllerFlash) {
      bigLeader.restoreFactoryDefaults();
      bigFollower.restoreFactoryDefaults();
      littleLeader.restoreFactoryDefaults();
      littleFollower.restoreFactoryDefaults();
    }

    bigFollower.follow(bigLeader, bigInvertFollower);
    littleFollower.follow(littleLeader, littleInvertFollower);
    bigLeader.setInverted(bigInvert);
    littleLeader.setInverted(littleInvert);
    bigLeader.setSmartCurrentLimit(30);
    littleLeader.setSmartCurrentLimit(30);
    bigFollower.setSmartCurrentLimit(30);
    littleFollower.setSmartCurrentLimit(30);
    bigLeader.enableVoltageCompensation(12.0);
    littleLeader.enableVoltageCompensation(12.0);
    bigFollower.enableVoltageCompensation(12.0);
    littleFollower.enableVoltageCompensation(12.0);


    bigEncoder = bigLeader.getEncoder();
    littleEncoder = littleLeader.getEncoder();
    bigPID = bigLeader.getPIDController();
    littlePID = littleLeader.getPIDController();

    bigLeader.setCANTimeout(0);
    bigFollower.setCANTimeout(0);
    littleLeader.setCANTimeout(0);
    littleFollower.setCANTimeout(0);

    if (Constants.burnMotorControllerFlash) {
      bigLeader.burnFlash();
      bigFollower.burnFlash();
      littleLeader.burnFlash();
      littleFollower.burnFlash();
    }
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.bigPositionRad =
        bigEncoder.getPosition() * (2.0 * Math.PI) / bigAfterEncoderReduction;
    inputs.bigVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bigEncoder.getVelocity())
            * (2.0 * Math.PI) / bigAfterEncoderReduction;
    inputs.bigAppliedVolts = bigLeader.getAppliedOutput();
    inputs.bigCurrentAmps = new double[] {bigLeader.getOutputCurrent(),
        bigFollower.getOutputCurrent()};
    inputs.bigTempCelcius = new double[] {bigLeader.getMotorTemperature(),
        bigFollower.getMotorTemperature()};

    inputs.littlePositionRad = littleEncoder.getPosition() * (2.0 * Math.PI)
        / littleAfterEncoderReduction;
    inputs.littleVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(littleEncoder.getVelocity())
            * (2.0 * Math.PI) / littleAfterEncoderReduction;
    inputs.littleAppliedVolts = littleLeader.getAppliedOutput();
    inputs.littleCurrentAmps = new double[] {littleLeader.getOutputCurrent(),
        littleFollower.getOutputCurrent()};
    inputs.littleTempCelcius = new double[] {littleLeader.getMotorTemperature(),
        littleFollower.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double bigVolts, double littleVolts) {
    bigLeader.setVoltage(bigVolts);
    littleLeader.setVoltage(littleVolts);
  }

  @Override
  public void setVelocity(double bigVelocityRadPerSec,
      double littleVelocityRadPerSec, double bigFFVolts, double littleFFVolts) {
    bigPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(bigVelocityRadPerSec)
            / bigAfterEncoderReduction,
        ControlType.kVelocity, 0, bigFFVolts, ArbFFUnits.kVoltage);
    littlePID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(littleVelocityRadPerSec)
            / littleAfterEncoderReduction,
        ControlType.kVelocity, 0, littleFFVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    bigLeader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    bigFollower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    littleLeader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    littleFollower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
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
