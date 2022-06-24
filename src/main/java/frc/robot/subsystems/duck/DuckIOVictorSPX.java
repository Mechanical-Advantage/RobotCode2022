// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class DuckIOVictorSPX implements DuckIO {
  private final VictorSPX motor;
  private final NetworkTableEntry playbackEntry =
      NetworkTableInstance.getDefault().getEntry("/quacker/playback");

  public DuckIOVictorSPX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        motor = new VictorSPX(10);
        break;
      default:
        throw new RuntimeException("Invalid robot for DuckIOVictorSPX!");
    }

    motor.configFactoryDefault();
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
  public void playSound(int id) {
    playbackEntry.forceSetDouble(id);
  }
}
