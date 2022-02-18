// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Tower extends SubsystemBase {
  private final TowerIO io;
  private final TowerIOInputs inputs = new TowerIOInputs();

  private final Alert lowerDisconnectedAlert =
      new Alert("Invalid data from lower cargo sensor. Is is connected?",
          AlertType.ERROR);
  private final Alert upperDisconnectedAlert =
      new Alert("Invalid data from upper cargo sensor. Is is connected?",
          AlertType.ERROR);

  /** Creates a new Tower. */
  public Tower(TowerIO io) {
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Tower", inputs);

    if (inputs.cargoSensorsAvailable) {
      if (inputs.lowerCargoSensor1 == inputs.lowerCargoSensor2) {
        lowerDisconnectedAlert.set(true);
      }
      if (inputs.upperCargoSensor1 == inputs.upperCargoSensor2) {
        upperDisconnectedAlert.set(true);
      }
    }

    Logger.getInstance().recordOutput("Tower/LowerCargoSensor",
        getLowerCargoSensor());
    Logger.getInstance().recordOutput("Tower/UpperCargoSensor",
        getUpperCargoSensor());
  }

  /** Run at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public void stop() {
    runPercent(0.0);
  }

  /** Returns whether the lower cargo sensor is tripped, defaults to false if disconnected. */
  public boolean getLowerCargoSensor() {
    return !inputs.lowerCargoSensor1;
  }

  /** Returns whether the upper cargo sensor is tripped, defaults to false if disconnected. */
  public boolean getUpperCargoSensor() {
    return !inputs.upperCargoSensor1;
  }
}
