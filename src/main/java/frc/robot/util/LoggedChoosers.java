// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlinkinLedDriver.BlinkinLedMode;

/** Manages all SendableChoosers, including replaying values without NT. */
public class LoggedChoosers extends SubsystemBase {

  private final SendableChooser<String> joystickModeChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> autoRoutineChooser =
      new SendableChooser<String>();
  private final SendableChooser<String> ledTestModeChooser =
      new SendableChooser<String>();

  private final ChooserData data = new ChooserData();

  public LoggedChoosers() {
    addOptions(joystickModeChooser,
        List.of("Curvature", "Split Arcade", "Tank"));
    addOptions(autoRoutineChooser,
        List.of("Do Nothing", "Five cargo (TD)", "Four cargo, standard (TD)",
            "Four cargo, cross field (TA)", "Three cargo (TD)",
            "Two cargo (TA)", "Two cargo (TC)", "Two cargo (TD)",
            "One cargo (TA)", "One cargo (TB)", "One cargo (TC)",
            "One cargo (TD)", "One cargo (FA)", "One cargo (FB)", "Taxi (TA)",
            "Taxi (TB)", "Taxi (TC)", "Taxi (TD)", "Taxi (FA)", "Taxi (FB)",
            "HP Practice", "Track Width Characterization",
            "FF Characterization (Drive/Forwards)",
            "FF Characterization (Drive/Backwards)",
            "FF Characterization (Flywheels/Forwards)",
            "FF Characterization (Flywheels/Backwards)"));

    SmartDashboard.putData("Joystick Mode", joystickModeChooser);
    SmartDashboard.putData("Auto Routine", autoRoutineChooser);

    if (LedSelector.testMode) {
      addOptions(ledTestModeChooser, Stream.of(BlinkinLedMode.values())
          .map(BlinkinLedMode::toString).collect(Collectors.toList()));
      SmartDashboard.putData("LED Test Mode", ledTestModeChooser);
    }
  }

  /** Adds a set of options to a SendableChooser. */
  private void addOptions(SendableChooser<String> chooser,
      List<String> options) {
    boolean firstOption = true;
    for (String option : options) {
      if (firstOption) {
        chooser.setDefaultOption(option, option);
        firstOption = false;
      } else {
        chooser.addOption(option, option);
      }
    }
  }

  /** Represents the selected values of all of the choosers. */
  private static class ChooserData implements LoggableInputs {
    public String joystickMode = "";
    public String autoRoutine = "";
    public String ledTestMode = "";

    @Override
    public void toLog(LogTable table) {
      table.put("JoystickMode", joystickMode);
      table.put("AutoRoutine", autoRoutine);
      table.put("LedTestMode", ledTestMode);
    }

    @Override
    public void fromLog(LogTable table) {
      joystickMode = table.getString("JoystickMode", joystickMode);
      autoRoutine = table.getString("AutoRoutine", autoRoutine);
      ledTestMode = table.getString("LedTestMode", ledTestMode);
    }
  }

  /** Updates chooser data when not replaying, processes data w/ logging framework. */
  @Override
  public void periodic() {
    if (!Logger.getInstance().hasReplaySource()) {
      data.joystickMode = joystickModeChooser.getSelected();
      data.autoRoutine = autoRoutineChooser.getSelected();
      if (LedSelector.testMode) {
        data.ledTestMode = ledTestModeChooser.getSelected();
      }
    }
    Logger.getInstance().processInputs("Choosers", data);
  }

  public String getJoystickMode() {
    return data.joystickMode;
  }

  public String getAutoRoutine() {
    return data.autoRoutine;
  }

  public String getLedTestMode() {
    return data.ledTestMode;
  }
}
