// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  /** Creates a new Feeder. */
  public Intake(IntakeIO io) {
    this.io = io;
    io.setRollerBrakeMode(false);
    io.setHopperBrakeMode(false);
    retract();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }

  /** Run the roller at the specified percentage. */
  public void runRollerPercent(double percent) {
    io.setRollerVoltage(percent * 12.0);
  }

  /** Run the hopper at the specified percentage. */
  public void runHopperPercent(double percent) {
    io.setHopperVoltage(percent * 12.0);
  }

  public void stop() {
    runRollerPercent(0.0);
    runHopperPercent(0.0);
  }

  public void extend() {
    io.setExtended(true);
  }

  public void retract() {
    io.setExtended(false);
  }
}
