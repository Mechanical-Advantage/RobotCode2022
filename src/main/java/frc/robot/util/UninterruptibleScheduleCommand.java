// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Schedules the given commands when this command is initialized. Useful for forking off from
 * CommandGroups. Note that if run from a CommandGroup, the group will not know about the status of
 * the scheduled commands, and will treat this command as finishing instantly.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class UninterruptibleScheduleCommand extends CommandBase {
  private final Set<Command> m_toSchedule;

  /**
   * Creates a new UninterruptibleScheduleCommand that schedules the given commands when
   * initialized.
   *
   * @param toSchedule the commands to schedule
   */
  public UninterruptibleScheduleCommand(Command... toSchedule) {
    m_toSchedule = Set.of(toSchedule);
  }

  @Override
  public void initialize() {
    for (Command command : m_toSchedule) {
      command.schedule(false);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
