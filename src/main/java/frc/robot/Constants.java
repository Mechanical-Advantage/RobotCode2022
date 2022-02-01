// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final Robot robot = Robot.ROBOT_KITBOT;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;
  public static final boolean burnMotorControllerFlash = false;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.",
          AlertType.ERROR);

  public static Robot getRobot() {
    if (RobotBase.isReal()) {
      if (robot == Robot.ROBOT_SIMBOT || robot == Robot.ROBOT_ROMI) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return Robot.ROBOT_2022C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
      case ROBOT_2020:
      case ROBOT_KITBOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
      case ROBOT_ROMI:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<Robot, String> logFolders = Map.of(Robot.ROBOT_2020,
      "/media/sda2/", Robot.ROBOT_KITBOT, "/media/sda1/");

  public static enum Robot {
    ROBOT_2022C, ROBOT_2022P, ROBOT_2020, ROBOT_KITBOT, ROBOT_SIMBOT, ROBOT_ROMI
  }

  public static enum Mode {
    REAL, REPLAY, SIM
  }
}
