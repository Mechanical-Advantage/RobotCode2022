// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class WaitForVision extends CommandBase {
  private static final double timeoutSecs = 0.5;

  private final Drive drive;
  private Timer timer = new Timer();

  /**
   * Creates a new WaitForVision. Waits until odometry is updated based on vision (or a timeout is
   * reached).
   */
  public WaitForVision(Drive drive) {
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getVisionResetComplete() || timer.hasElapsed(timeoutSecs);
  }
}
