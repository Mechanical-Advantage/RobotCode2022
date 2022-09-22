// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.subsystems.drive.Drive;

public class CrazyDuck extends SequentialCommandGroup {
  public static final Pose2d firstCargoPosition =
      AutoPosition.TARMAC_A_FAR.getPose().transformBy(new Transform2d(
          new Translation2d(1.4, -0.1), Rotation2d.fromDegrees(-20.0)));
  public static final Pose2d firstCargoBackUpPosition =
      AutoPosition.TARMAC_A_FAR.getPose().transformBy(new Transform2d(
          new Translation2d(0.0, 0.2), Rotation2d.fromDegrees(-70.0)));
  public static final Pose2d cargoCollisionPosition =
      FieldConstants.cargoDOpposite.transformBy(new Transform2d(
          new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-45.0)));
  public static final Pose2d endPosition =
      new Pose2d(new Translation2d(FieldConstants.fieldLength - 1.0,
          FieldConstants.fieldWidth / 2.0), Rotation2d.fromDegrees(-45.0));

  /** Creates a new CrazyDuck. */
  public CrazyDuck(Drive drive, RobotState robotState) {
    addCommands(
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(AutoPosition.TARMAC_A_FAR.getPose(), firstCargoPosition),
            0.0, false),
        new MotionProfileCommand(drive, robotState, 0.0,
            List.of(firstCargoPosition, firstCargoBackUpPosition), 0.0, true),
        new MotionProfileCommand(drive, robotState, 0.0, List
            .of(firstCargoBackUpPosition, cargoCollisionPosition, endPosition),
            0.0, false));
  }
}
