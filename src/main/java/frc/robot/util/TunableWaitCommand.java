// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that does nothing but takes a specified amount of time to finish, with the duration
 * provided by a TunableNumber. Useful for CommandGroups. Can also be subclassed to make a command
 * with an internal timer.
 */
public class TunableWaitCommand extends CommandBase {
  protected Timer m_timer = new Timer();
  private final TunableNumber m_duration;

  /**
   * Creates a new TunableWaitCommand. This command will do nothing, and end after the specified
   * duration (provided by a TunableNumber).
   *
   * @param seconds the time to wait, in seconds
   */
  public TunableWaitCommand(TunableNumber seconds) {
    m_duration = seconds;
    SendableRegistry.setName(this,
        getName() + ": " + seconds.get() + " seconds");
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration.get());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
