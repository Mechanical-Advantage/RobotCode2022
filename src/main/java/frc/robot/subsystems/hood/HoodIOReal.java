// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class HoodIOReal implements HoodIO {
  private final DoubleSolenoid solenoid;

  public HoodIOReal() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        solenoid = new DoubleSolenoid(Pneumatics.revModuleID,
            PneumaticsModuleType.REVPH, 3, 6);
        break;
      default:
        throw new RuntimeException("Invalid robot for HoodIOReal!");
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.raised = solenoid.get() == Value.kForward;
  }

  @Override
  public void setRaised(boolean raised) {
    solenoid.set(raised ? Value.kForward : Value.kReverse);
  }
}
