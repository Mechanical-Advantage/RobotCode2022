// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.kicker.KickerIO.KickerIOInputs;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputs inputs = new KickerIOInputs();

  /** Creates a new Kicker. */
  public Kicker(KickerIO io) {
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Kicker", inputs);
  }

  /** Run at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public void stop() {
    runPercent(0.0);
  }
}
