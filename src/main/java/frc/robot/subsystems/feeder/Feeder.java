// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Feeder extends SubsystemBase {
  private static final int proxSensorFaultCycles = 3; // Send alert after invalid data for this many
                                                      // cycles
  private static final double colorSensorThreshold = 1.5;

  private final FeederIO io;
  private final FeederIOInputs inputs = new FeederIOInputs();

  private Leds leds;
  private Supplier<Boolean> colorSensorDisableSupplier = () -> false;
  private double towerShootSpeed = 0.0;

  private int lowerProxSensorFaultCounter;
  private int upperProxSensorFaultCounter;
  private final Alert lowerProxDisconnectedAlert =
      new Alert("Invalid data from lower cargo sensor. Is is connected?",
          AlertType.ERROR);
  private final Alert upperProxDisconnectedAlert =
      new Alert("Invalid data from upper cargo sensor. Is is connected?",
          AlertType.ERROR);

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    this.io = io;
    io.setHopperBrakeMode(false);
    io.setTowerBrakeMode(false);
    io.setKickerBrakeMode(false);
  }

  public void setLeds(Leds leds) {
    this.leds = leds;
  }

  public void setOverride(Supplier<Boolean> colorSensorDisableSupplier) {
    this.colorSensorDisableSupplier = colorSensorDisableSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Feeder", inputs);

    // Check for prox sensor faults
    if (inputs.proxSensorsAvailable) {
      if (inputs.lowerProxSensor1 == inputs.lowerProxSensor2) {
        lowerProxSensorFaultCounter += 1;
        if (lowerProxSensorFaultCounter >= proxSensorFaultCycles) {
          lowerProxDisconnectedAlert.set(true);
        }
      } else {
        lowerProxSensorFaultCounter = 0;
      }

      if (inputs.upperProxSensor1 == inputs.upperProxSensor2) {
        upperProxSensorFaultCounter += 1;
        if (upperProxSensorFaultCounter >= proxSensorFaultCycles) {
          upperProxDisconnectedAlert.set(true);
        }
      } else {
        upperProxSensorFaultCounter = 0;
      }
    }

    // Log prox sensor states
    Logger.getInstance().recordOutput("Tower/LowerProxSensor",
        getLowerProxSensor());
    Logger.getInstance().recordOutput("Tower/UpperProxSensor",
        getUpperProxSensor());
    SmartDashboard.putBoolean("Feeder/One Cargo", getUpperProxSensor());
    SmartDashboard.putBoolean("Feeder/Two Cargo",
        getUpperProxSensor() && getLowerProxSensor());

    // Set cargo count for LEDs
    if (getUpperProxSensor() && getLowerProxSensor()) {
      leds.setTowerCount(2);
    } else if (getUpperProxSensor()) {
      leds.setTowerCount(1);
    } else {
      leds.setTowerCount(0);
    }

    // Update color sensor value
    boolean hasWrongColor = false;
    if (!colorSensorDisableSupplier.get()) {
      switch (DriverStation.getAlliance()) {
        case Red:
          hasWrongColor = inputs.colorSensorBlue > inputs.colorSensorRed
              * colorSensorThreshold;
          break;
        case Blue:
          hasWrongColor = inputs.colorSensorRed > inputs.colorSensorBlue
              * colorSensorThreshold;
          break;
        default:
          break;
      }
    }
  }

  public void requestTowerShootPercent(double percent) {
    towerShootSpeed = percent;
  }

  /** Returns whether the lower prox sensor is tripped, defaults to false if disconnected. */
  private boolean getLowerProxSensor() {
    return !inputs.lowerProxSensor1;
  }

  /** Returns whether the upper prox sensor is tripped, defaults to false if disconnected. */
  private boolean getUpperProxSensor() {
    return !inputs.upperProxSensor1;
  }
}
