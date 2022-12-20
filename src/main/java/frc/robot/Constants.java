// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_2022C;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT || robot == RobotType.ROBOT_ROMI) { // Invalid robot
        // selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2022C;
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

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2022C, "/media/sda1/", RobotType.ROBOT_2020, "/media/sda2/");

  public static enum RobotType {
    ROBOT_2022C,
    ROBOT_2022P,
    ROBOT_2020,
    ROBOT_KITBOT,
    ROBOT_SIMBOT,
    ROBOT_ROMI
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }
}
