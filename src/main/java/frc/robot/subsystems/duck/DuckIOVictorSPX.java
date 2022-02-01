// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;

public class DuckIOVictorSPX implements DuckIO {

  private final VictorSPX motor;

  public DuckIOVictorSPX() {
    switch (Constants.getRobot()) {
      case ROBOT_KITBOT:
        motor = new VictorSPX(10);
        break;
      default:
        throw new RuntimeException("Invalid robot for DuckIOVictorSPX!");
    }

    if (Constants.burnMotorControllerFlash) {
      motor.configFactoryDefault();
    }

    motor.setInverted(false);
    motor.configVoltageCompSaturation(12.0);
  }

  @Override
  public void updateInputs(DuckIOInputs inputs) {
    inputs.appliedVolts = motor.getMotorOutputVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    motor.set(ControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }
}
