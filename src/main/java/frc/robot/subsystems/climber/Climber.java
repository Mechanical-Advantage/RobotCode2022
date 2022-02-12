// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Climber", inputs);

    if (DriverStation.isDisabled()) {
      if (!inputs.locked) {
        lock();
      }
    }
  }

  public void runPercent(double percent) {
    if (!inputs.locked) {
      io.setVoltage(percent * 12.0);
    }
  }

  public void lock() {
    io.setLocked(true);
    io.setVoltage(0.0);
  }

  public void unlock() {
    io.setLocked(false);
  }
}
