// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
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

  private Leds leds;
  private Supplier<Boolean> cargoSensorDisableSupplier = () -> false;
  private double shootSpeed = 0.0;
  private int lowerSensorsEqual;
  private int upperSensorsEqual;
  private final int cyclesTripped = 3;


  /** Creates a new Tower. */
  public Tower(TowerIO io) {
    this.io = io;
    io.setBrakeMode(false);
  }

  public void setLeds(Leds leds) {
    this.leds = leds;
  }

  public void setOverride(Supplier<Boolean> cargoSensorDisableSupplier) {
    this.cargoSensorDisableSupplier = cargoSensorDisableSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Tower", inputs);



    if (inputs.cargoSensorsAvailable) {
      if (inputs.lowerCargoSensor1 == inputs.lowerCargoSensor2) {
        lowerSensorsEqual += 1;
        if (lowerSensorsEqual >= cyclesTripped) {
          lowerDisconnectedAlert.set(true);
          System.out.println("Alert");
        }
      } else {
        lowerSensorsEqual = 0;
      }
      if (inputs.upperCargoSensor1 == inputs.upperCargoSensor2) {
        upperSensorsEqual += 1;
        if (upperSensorsEqual >= cyclesTripped) {
          upperDisconnectedAlert.set(true);
          System.out.println("Alert2");
        } else {
          upperSensorsEqual = 0;
        }

      }
    }

    Logger.getInstance().recordOutput("Tower/LowerCargoSensor",
        getLowerCargoSensor());
    Logger.getInstance().recordOutput("Tower/UpperCargoSensor",
        getUpperCargoSensor());

    SmartDashboard.putBoolean("Tower/One Cargo", getUpperCargoSensor());
    SmartDashboard.putBoolean("Tower/Two Cargo",
        getUpperCargoSensor() && getLowerCargoSensor());
    leds.setTowerFull(getUpperCargoSensor() && getLowerCargoSensor());
  }

  /** Run at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public void requestShootPercent(double percent) {
    shootSpeed = percent;
  }

  public void runShootSpeed() {
    runPercent(shootSpeed);
  }

  public void stop() {
    runPercent(0.0);
  }

  /** Returns whether the lower cargo sensor is tripped, defaults to false if disconnected. */
  public boolean getLowerCargoSensor() {
    return !cargoSensorDisableSupplier.get() && !inputs.lowerCargoSensor1;
  }

  /** Returns whether the upper cargo sensor is tripped, defaults to false if disconnected. */
  public boolean getUpperCargoSensor() {
    return !cargoSensorDisableSupplier.get() && !inputs.upperCargoSensor1;
  }
}
