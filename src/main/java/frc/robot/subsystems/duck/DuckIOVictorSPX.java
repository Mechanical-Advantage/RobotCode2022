// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;
import frc.robot.subsystems.duck.Duck.DuckSound;

public class DuckIOVictorSPX implements DuckIO {
  private final VictorSPX motor;
  private final Map<DuckSound, DigitalOutput> sounds;

  public DuckIOVictorSPX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022P:
      case ROBOT_SIMBOT:
        motor = new VictorSPX(10);
        sounds = new HashMap<>();
        sounds.put(DuckSound.MATCH_START, new DigitalOutput(0));
        sounds.put(DuckSound.QUACK_1, new DigitalOutput(1));
        sounds.put(DuckSound.QUACK_2, new DigitalOutput(2));
        sounds.put(DuckSound.QUACK_3, new DigitalOutput(3));
        sounds.put(DuckSound.QUACK_4, new DigitalOutput(4));
        sounds.put(DuckSound.QUACK_5, new DigitalOutput(5));
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
  public void setActive(DuckSound sound) {
    for (Map.Entry<DuckSound, DigitalOutput> entry : sounds.entrySet()) {
      entry.getValue().set(entry.getKey() != sound); // Down to activate
    }
  }
}
