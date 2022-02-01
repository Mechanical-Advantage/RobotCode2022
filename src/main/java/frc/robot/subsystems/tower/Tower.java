// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;

public class Tower extends SubsystemBase {
  private final TowerIO io;
  private final TowerIOInputs inputs = new TowerIOInputs();

  /** Creates a new Tower. */
  public Tower(TowerIO io) {
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Tower", inputs);
  }

  /** Run at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public void stop() {
    runPercent(0.0);
  }

  public boolean getCargoSensorTripped() {
    return !inputs.cargoSensor;
  }
}
