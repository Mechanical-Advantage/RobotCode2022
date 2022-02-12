// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.Constants;

public class PneumaticsIOCTRE implements PneumaticsIO {
  private static final double supplyNormalized = 4.5868055556;

  private final PneumaticsControlModule pneumatics;
  private final AnalogInput sensor;

  public PneumaticsIOCTRE() {
    switch (Constants.getRobot()) {
      case ROBOT_2020:
        pneumatics = new PneumaticsControlModule();
        sensor = new AnalogInput(0);
        break;
      default:
        throw new RuntimeException("Invalid robot for PneumaticsIOCTRE!");
    }

    sensor.setAverageBits(4);
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    double pressure =
        ((sensor.getAverageVoltage() / supplyNormalized) * 250) - 25;
    inputs.pressurePsi = pressure < 0 ? 0 : pressure;
    inputs.compressorActive = pneumatics.getCompressor();
    inputs.compressorCurrentAmps = pneumatics.getCompressorCurrent();
  }
}
