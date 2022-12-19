// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    inputs.pressurePsi = ((sensor.getAverageVoltage() / supplyNormalized) * 250) - 25;
    inputs.compressorActive = pneumatics.getCompressor();
    inputs.compressorCurrentAmps = pneumatics.getCompressorCurrent();
  }
}
