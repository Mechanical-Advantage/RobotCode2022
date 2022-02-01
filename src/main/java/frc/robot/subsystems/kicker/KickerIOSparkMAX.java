// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class KickerIOSparkMAX implements KickerIO {
  private boolean invert = false;
  private boolean invertFollower = false;
  private double afterEncoderReduction = 1.0;

  private final CANSparkMax leader;
  private final CANSparkMax follower;
  private final RelativeEncoder encoder;

  public KickerIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        leader = new CANSparkMax(0, MotorType.kBrushless);
        follower = new CANSparkMax(1, MotorType.kBrushless);
        invert = false;
        invertFollower = false;
        afterEncoderReduction = 1.0;
        break;
      default:
        throw new RuntimeException("Invalid robot for KickerIOSparkMax!");
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

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);

    if (Constants.burnMotorControllerFlash) {
      leader.burnFlash();
      follower.burnFlash();
    }
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.positionRad =
        encoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            * (2.0 * Math.PI) / afterEncoderReduction;
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
  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
