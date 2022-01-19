// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticLauncher extends SubsystemBase {
  private static final List<Integer> solenoidChannels = List.of(0, 1, 2, 3);
  private List<Solenoid> solenoids = new ArrayList<>();

  /** Creates a new PneumaticLauncher. */
  public PneumaticLauncher() {
    SmartDashboard.putNumber("LauncherLengthMs", 20.0);
    for (Integer channel : solenoidChannels) {
      solenoids.add(new Solenoid(PneumaticsModuleType.CTREPCM, channel));
    }
  }

  @Override
  public void periodic() {}

  public void setExtended(boolean extended) {
    for (Solenoid solenoid : solenoids) {
      solenoid.set(extended);
    }
  }

  public void runPulse() {
    double lengthSecs =
        SmartDashboard.getNumber("LauncherLengthMs", 0.0) / 1000.0;
    for (Solenoid solenoid : solenoids) {
      solenoid.setPulseDuration(lengthSecs);
      solenoid.startPulse();
    }
  }
}
