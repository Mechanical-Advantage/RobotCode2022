// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.Constants;

public class PneumaticsIOREV implements PneumaticsIO {
  private final PneumaticHub pneumatics;

  public PneumaticsIOREV() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        pneumatics = new PneumaticHub(Pneumatics.revModuleID);
        break;
      default:
        throw new RuntimeException("Invalid robot for PneumaticsIOREV!");
    }

    useLowClosedLoopThresholds(false);
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    inputs.pressurePsi = pneumatics.getPressure(0);
    inputs.compressorActive = pneumatics.getCompressor();
    inputs.compressorCurrentAmps = pneumatics.getCompressorCurrent();
  }

  @Override
  public void useLowClosedLoopThresholds(boolean useLow) {
    if (useLow) {
      pneumatics.enableCompressorAnalog(50, 60);
    } else {
      pneumatics.enableCompressorAnalog(80, 120);
    }
  }
}
