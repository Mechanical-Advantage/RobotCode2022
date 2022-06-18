// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.duck.DuckIO.DuckIOInputs;

public class Duck extends SubsystemBase {
  private final DuckIO io;
  private final DuckIOInputs inputs = new DuckIOInputs();

  /** Creates a new Duck. */
  public Duck(DuckIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Duck", inputs);
  }

  /** Run the duck at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }
}
