// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticLauncher;

public class LaunchPulse extends CommandBase {
  private final PneumaticLauncher launcher;
  private int cycleCount;

  /** Creates a new LaunchPulse. */
  public LaunchPulse(PneumaticLauncher launcher) {
    this.launcher = launcher;
    SmartDashboard.putNumber("LauncherLengthCycles", 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.setExtended(true);
    cycleCount = -1; // First execute call occurs during the same cycle
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cycleCount++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.setExtended(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cycleCount >= SmartDashboard.getNumber("LauncherLengthCycles", 1);
  }
}
