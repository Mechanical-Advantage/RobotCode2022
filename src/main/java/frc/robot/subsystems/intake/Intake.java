// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  /** Creates a new Feeder. */
  public Intake(IntakeIO io) {
    this.io = io;
    io.setBrakeMode(false);
    retract();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }

  /** Run the roller at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public void stop() {
    runPercent(0.0);
  }

  public void extend() {
    io.setExtended(true);
  }

  public void retract() {
    io.setExtended(false);
  }
}
