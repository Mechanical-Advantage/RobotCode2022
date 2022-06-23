// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.subsystems.drive.Drive;

public class DuckToHangar extends SequentialCommandGroup {
  /** Creates a new DuckToHangar. Taxis to the hangar then spins in place. */
  public DuckToHangar(boolean startAtFender, Drive drive,
      RobotState robotState) {
    addCommands((startAtFender ? new WaitCommand(5.0) : new InstantCommand()),
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(
                startAtFender ? AutoPosition.FENDER_A.getPose()
                    : AutoPosition.TARMAC_B.getPose(),
                new Pose2d(FieldConstants.hangarLength / 2.0,
                    FieldConstants.fieldWidth - FieldConstants.hangarWidth
                        + 0.5,
                    Rotation2d.fromDegrees(90.0)),
                new Pose2d(FieldConstants.hangarLength / 2.0,
                    FieldConstants.fieldWidth
                        - (FieldConstants.hangarWidth / 2.0),
                    Rotation2d.fromDegrees(90.0))),
            0.0, false,
            List.of(new MaxVelocityConstraint(Units.inchesToMeters(100.0)))),
        new WaitCommand(1.0), new Spin(drive));
  }
}
