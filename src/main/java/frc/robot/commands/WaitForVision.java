// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class WaitForVision extends CommandBase {
  private static final double timeoutSecs = 0.5;

  private final RobotState robotState;
  private Timer timer = new Timer();

  /**
   * Creates a new WaitForVision. Waits until odometry is updated based on vision (or a timeout is
   * reached).
   */
  public WaitForVision(RobotState robotState) {
    this.robotState = robotState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/WaitForVision", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotState.getVisionResetComplete() || timer.hasElapsed(timeoutSecs);
  }
}
