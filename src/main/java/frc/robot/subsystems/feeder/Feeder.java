// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PrepareShooterPreset;
import frc.robot.commands.PrepareShooterPreset.ShooterPreset;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.Alert;
import frc.robot.util.TunableNumber;
import frc.robot.util.Alert.AlertType;

public class Feeder extends SubsystemBase {
  private static final int proxSensorFaultCycles = 3; // Send alert after invalid data for this many
                                                      // cycles
  private static final double colorSensorChannelThreshold = 1.5;
  private static final int colorSensorProxThreshold = 300;

  private final TunableNumber shootKickerPercent =
      new TunableNumber("Feeder/SHOOT/Kicker", 0.3);

  private final TunableNumber intakeForwardsHopperPercent =
      new TunableNumber("Feeder/INTAKE_FORWARDS/Hopper", 1.0);
  private final TunableNumber intakeForwardsTowerPercent =
      new TunableNumber("Feeder/INTAKE_FORWARDS/Tower", 0.5);
  private final TunableNumber intakeForwardsKickerPercent =
      new TunableNumber("Feeder/INTAKE_FORWARDS/Kicker", -0.3);

  private final TunableNumber intakeBackwardsHopperPercent =
      new TunableNumber("Feeder/INTAKE_BACKWARDS/Hopper", -1.0);
  private final TunableNumber intakeBackwardsTowerPercent =
      new TunableNumber("Feeder/INTAKE_BACKWARDS/Tower", -0.5);
  private final TunableNumber intakeBackwardsKickerPercent =
      new TunableNumber("Feeder/INTAKE_BACKWARDS/Kicker", -0.3);

  private final TunableNumber ejectBottomIntakePercent =
      new TunableNumber("Feeder/EJECT_BOTTOM/Intake", -1.0);
  private final TunableNumber ejectBottomHopperPercent =
      new TunableNumber("Feeder/EJECT_BOTTOM/Hopper", -1.0);
  private final TunableNumber ejectBottomTowerPercent =
      new TunableNumber("Feeder/EJECT_BOTTOM/Tower", -0.5);

  private final TunableNumber ejectTopHopperPercent =
      new TunableNumber("Feeder/EJECT_TOP/Hopper", 1.0);
  private final TunableNumber ejectTopTowerPercent =
      new TunableNumber("Feeder/EJECT_TOP/Tower", 0.6);
  private final TunableNumber ejectTopKickerPercent =
      new TunableNumber("Feeder/EJECT_TOP/Kicker", 0.3);

  private final TunableNumber ejectBottomAllDuration =
      new TunableNumber("Feeder/EJECT_BOTTOM_ALL/Duration", 0.12);
  private final TunableNumber ejectBottomFinishDuration =
      new TunableNumber("Feeder/EJECT_BOTTOM_FINISH/Duration", 0.4);
  private final TunableNumber ejectTopFinishDuration =
      new TunableNumber("Feeder/EJECT_TOP_FINISH/Duration", 0.45);

  private final FeederIO io;
  private final FeederIOInputs inputs = new FeederIOInputs();

  private Leds leds;
  private Supplier<Boolean> colorSensorDisableSupplier = () -> false;
  private boolean shootRequested = false;
  private boolean intakeForwardsRequested = false;
  private boolean intakeBackwardsRequested = false;
  private double towerShootSpeed = 0.0;

  private FeederState state = FeederState.IDLE;
  private Timer stateTimer = new Timer();
  private boolean intakingProxValue = false;
  private int intakingProxCount = 0;

  private Flywheels flywheels;
  private Hood hood;
  private Command intakeCommand = new InstantCommand();
  private Command prepareShooterCommand = new InstantCommand();

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
    stateTimer.start();
    stateTimer.reset();
  }

  public void setSubsystems(Leds leds, Intake intake, Flywheels flywheels,
      Hood hood) {
    this.leds = leds;
    this.flywheels = flywheels;
    this.hood = hood;
    intakeCommand = new StartEndCommand(
        () -> intake.runPercent(ejectBottomIntakePercent.get()), intake::stop,
        intake);
    prepareShooterCommand = new PrepareShooterPreset(flywheels, hood, this,
        ShooterPreset.OPPONENT_EJECT);
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
    if (!colorSensorDisableSupplier.get() && DriverStation.isTeleop()) {
      switch (DriverStation.getAlliance()) {
        case Red:
          hasWrongColor = inputs.colorSensorBlue > inputs.colorSensorRed
              * colorSensorChannelThreshold;
          break;
        case Blue:
          hasWrongColor = inputs.colorSensorRed > inputs.colorSensorBlue
              * colorSensorChannelThreshold;
          break;
        default:
          break;
      }
    }

    // Run the state machine
    double hopperPercent = 0.0;
    double towerPercent = 0.0;
    double kickerPercent = 0.0;
    if (DriverStation.isDisabled()) {
      if (state != FeederState.IDLE) {
        setState(FeederState.IDLE);
      }
    } else {
      switch (state) {
        case IDLE:
          if (shootRequested) {
            setState(FeederState.SHOOT);
          } else if (intakeForwardsRequested) {
            setState(FeederState.INTAKE_FORWARDS);
            intakingProxValue = false;
            intakingProxCount = 0;
          } else if (intakeBackwardsRequested) {
            setState(FeederState.INTAKE_BACKWARDS);
          }
          break;
        case SHOOT:
          towerPercent = towerShootSpeed;
          kickerPercent = shootKickerPercent.get();
          if (!shootRequested) {
            setState(FeederState.IDLE);
          }
          break;
        case INTAKE_FORWARDS:
          // Count balls
          if (getLowerProxSensor() && !intakingProxValue) {
            intakingProxCount++;
          }
          intakingProxValue = getLowerProxSensor();

          // Select next state
          if (shootRequested) {
            setState(FeederState.SHOOT);
          } else if (!intakeForwardsRequested) {
            setState(FeederState.IDLE);
          } else if (getLowerProxSensor() && hasWrongColor) { // Time to eject
            if (getUpperProxSensor() || intakingProxCount >= 2) { // Second cargo, eject bottom
              setState(FeederState.EJECT_BOTTOM_ALL);
              intakeCommand.schedule(false);
            } else { // First cargo, eject top
              setState(FeederState.EJECT_TOP_WAIT);
              prepareShooterCommand.schedule(false);
            }
          } else if (getLowerProxSensor() && getUpperProxSensor()) {
            setState(FeederState.IDLE);
            break; // Break out before speeds are set to avoid toggle
          }

          // Set speeds at the end to avoid toggle
          hopperPercent = intakeForwardsHopperPercent.get();
          towerPercent = intakeForwardsTowerPercent.get();
          kickerPercent = intakeForwardsKickerPercent.get();
          break;
        case INTAKE_BACKWARDS:
          hopperPercent = intakeBackwardsHopperPercent.get();
          towerPercent = intakeBackwardsTowerPercent.get();
          kickerPercent = intakeBackwardsKickerPercent.get();
          if (shootRequested) {
            setState(FeederState.SHOOT);
          } else if (!intakeBackwardsRequested) {
            setState(FeederState.IDLE);
          }
          break;
        case EJECT_BOTTOM_ALL:
          hopperPercent = ejectBottomHopperPercent.get();
          towerPercent = ejectBottomTowerPercent.get();
          if (stateTimer.hasElapsed(ejectBottomAllDuration.get())) {
            setState(FeederState.EJECT_BOTTOM_FINISH);
          }
          break;
        case EJECT_BOTTOM_FINISH:
          hopperPercent = ejectBottomHopperPercent.get();
          if (stateTimer.hasElapsed(ejectBottomFinishDuration.get())) {
            setState(FeederState.IDLE);
            intakeCommand.cancel();
          }
          break;
        case EJECT_TOP_WAIT:
          if (flywheels.atGoal() && hood.atGoal()) {
            setState(FeederState.EJECT_TOP_FINISH);
          }
          break;
        case EJECT_TOP_FINISH:
          hopperPercent = ejectTopHopperPercent.get();
          towerPercent = ejectTopTowerPercent.get();
          kickerPercent = ejectTopKickerPercent.get();
          if (getLowerProxSensor() && hasWrongColor) {
            stateTimer.reset();
          }
          if (stateTimer.hasElapsed(ejectTopFinishDuration.get())) {
            setState(FeederState.IDLE);
            prepareShooterCommand.cancel();
          }
          break;
        default:
          break;
      }

      // Update voltages
      io.setHopperVoltage(hopperPercent * 12.0);
      io.setTowerVoltage(towerPercent * 12.0);
      io.setKickerVoltage(kickerPercent * 12.0);
    }

    // Log current state
    Logger.getInstance().recordOutput("Feeder/State", state.toString());
    Logger.getInstance().recordOutput("Feeder/StateTimer", stateTimer.get());
  }

  private void setState(FeederState newState) {
    state = newState;
    stateTimer.reset();
  }

  public void requestShoot(boolean active) {
    shootRequested = active;
  }

  public void requestIntakeForwards(boolean active) {
    intakeForwardsRequested = active;
  }

  public void requestIntakeBackwards(boolean active) {
    intakeBackwardsRequested = active;
  }

  public void requestTowerShootPercent(double percent) {
    towerShootSpeed = percent;
  }

  /** Returns whether the lower prox sensor is tripped, defaults to false if disconnected. */
  public boolean getLowerProxSensor() {
    if (colorSensorDisableSupplier.get()) {
      return !inputs.lowerProxSensor1;
    } else {
      return inputs.colorSensorProx > colorSensorProxThreshold;
    }
  }

  /** Returns whether the upper prox sensor is tripped, defaults to false if disconnected. */
  public boolean getUpperProxSensor() {
    return !inputs.upperProxSensor1;
  }

  private static enum FeederState {
    IDLE, SHOOT, INTAKE_FORWARDS, INTAKE_BACKWARDS, EJECT_BOTTOM_ALL, EJECT_BOTTOM_FINISH, EJECT_TOP_WAIT, EJECT_TOP_FINISH
  }
}
