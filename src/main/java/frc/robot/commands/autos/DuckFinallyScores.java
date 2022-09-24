// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;

public class DuckFinallyScores extends SequentialCommandGroup {
  public static final Pose2d recoilPosition = AutoPosition.FENDER_B_REVERSED
      .getPose().transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0));
  public static final Pose2d crashPosition = AutoPosition.FENDER_B_REVERSED
      .getPose().transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
  public static final Pose2d taxiPosition = AutoPosition.FENDER_B_REVERSED
      .getPose().transformBy(GeomUtil.transformFromTranslation(-2.5, 0.0));


  /** Creates a new DuckFinallyScores. */
  public DuckFinallyScores(Drive drive, RobotState robotState) {
    addCommands(new WaitCommand(8.0),
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(AutoPosition.FENDER_B_REVERSED.getPose(), recoilPosition),
            0.0, true),
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(recoilPosition, crashPosition), Double.POSITIVE_INFINITY,
            false),
        new WaitCommand(2.0), new MotionProfileCommand(drive, robotState, 0.0,
            List.of(crashPosition, taxiPosition), 0.0, true));
  }
}
