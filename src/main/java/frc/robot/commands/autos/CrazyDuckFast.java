// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.RobotState;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.subsystems.drive.Drive;

public class CrazyDuckFast extends SequentialCommandGroup {

  /** Creates a new CrazyDuckFast. */
  public CrazyDuckFast(Drive drive, RobotState robotState) {
    addCommands(new WaitCommand(1.0),
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(AutoPosition.TARMAC_A_FAR_TURNED.getPose(),
                CrazyDuck.cargoCollisionPosition, CrazyDuck.endPosition),
            0.0, false));
  }
}
