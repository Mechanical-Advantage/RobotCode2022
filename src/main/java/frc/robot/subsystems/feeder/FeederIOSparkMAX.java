// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.PicoColorSensor;
import frc.robot.util.PicoColorSensor.RawColor;
import frc.robot.util.SparkMAXBurnManager;

public class FeederIOSparkMAX implements FeederIO {
  private boolean hopperInvert = false;
  private boolean towerInvert = false;
  private boolean kickerInvert = false;
  private double hopperAfterEncoderReduction = 1.0;
  private double towerAfterEncoderReduction = 1.0;
  private double kickerAfterEncoderReduction = 1.0;

  private final CANSparkMax hopperMotor;
  private final CANSparkMax towerMotor;
  private final CANSparkMax kickerMotor;
  private final RelativeEncoder hopperEncoder;
  private final RelativeEncoder towerEncoder;
  private final RelativeEncoder kickerEncoder;

  private final DigitalInput lowerProxSensor1;
  private final DigitalInput lowerProxSensor2;
  private final DigitalInput upperProxSensor1;
  private final DigitalInput upperProxSensor2;
  private final PicoColorSensor colorSensor;

  public FeederIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        hopperMotor = new CANSparkMax(13, MotorType.kBrushless);
        hopperInvert = false;
        hopperAfterEncoderReduction = 9.0 * (48.0 / 24.0);

        towerMotor = new CANSparkMax(30, MotorType.kBrushless);
        towerInvert = false;
        towerAfterEncoderReduction = 9.0;

        kickerMotor = new CANSparkMax(1, MotorType.kBrushless);
        kickerInvert = true;
        kickerAfterEncoderReduction = 60.0 / 12.0;

        lowerProxSensor1 = new DigitalInput(4);
        lowerProxSensor2 = new DigitalInput(5);
        upperProxSensor1 = new DigitalInput(6);
        upperProxSensor2 = new DigitalInput(7);
        colorSensor = new PicoColorSensor();
        break;
      default:
        throw new RuntimeException("Invalid robot for FeederIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      hopperMotor.restoreFactoryDefaults();
      towerMotor.restoreFactoryDefaults();
      kickerMotor.restoreFactoryDefaults();
    }

    hopperMotor.setInverted(hopperInvert);
    hopperMotor.setSmartCurrentLimit(20);
    hopperMotor.enableVoltageCompensation(12.0);

    towerMotor.setInverted(towerInvert);
    towerMotor.setSmartCurrentLimit(25);
    towerMotor.enableVoltageCompensation(12.0);

    kickerMotor.setInverted(kickerInvert);
    kickerMotor.setSmartCurrentLimit(20);
    kickerMotor.enableVoltageCompensation(12.0);

    hopperEncoder = hopperMotor.getEncoder();
    towerEncoder = towerMotor.getEncoder();
    kickerEncoder = kickerMotor.getEncoder();

    hopperMotor.setCANTimeout(0);
    towerMotor.setCANTimeout(0);
    kickerMotor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      hopperMotor.burnFlash();
      towerMotor.burnFlash();
      kickerMotor.burnFlash();
    }
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.lowerProxSensor1 = lowerProxSensor1.get();
    inputs.lowerProxSensor2 = lowerProxSensor2.get();
    inputs.upperProxSensor1 = upperProxSensor1.get();
    inputs.upperProxSensor2 = upperProxSensor2.get();

    RawColor color = colorSensor.getRawColor0();
    inputs.colorSensorConnected = colorSensor.isSensor0Connected();
    inputs.colorSensorRed = color.red;
    inputs.colorSensorGreen = color.blue; // Green and blue channels swapped
    inputs.colorSensorBlue = color.green;
    inputs.colorSensorProx = colorSensor.getProximity0();

    inputs.hopperPositionRad =
        Units.rotationsToRadians(hopperEncoder.getPosition()) / hopperAfterEncoderReduction;
    inputs.hopperVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(hopperEncoder.getVelocity())
            / hopperAfterEncoderReduction;
    inputs.hopperAppliedVolts =
        hopperMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.hopperCurrentAmps = new double[] {hopperMotor.getOutputCurrent()};
    inputs.hopperTempCelcius = new double[] {hopperMotor.getMotorTemperature()};

    inputs.towerPositionRad =
        Units.rotationsToRadians(towerEncoder.getPosition()) / towerAfterEncoderReduction;
    inputs.towerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(towerEncoder.getVelocity())
            / towerAfterEncoderReduction;
    inputs.towerAppliedVolts = towerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.towerCurrentAmps = new double[] {towerMotor.getOutputCurrent()};
    inputs.towerTempCelcius = new double[] {towerMotor.getMotorTemperature()};

    inputs.kickerPositionRad =
        Units.rotationsToRadians(kickerEncoder.getPosition()) / kickerAfterEncoderReduction;
    inputs.kickerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(kickerEncoder.getVelocity())
            / kickerAfterEncoderReduction;
    inputs.kickerAppliedVolts =
        kickerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.kickerCurrentAmps = new double[] {kickerMotor.getOutputCurrent()};
    inputs.kickerTempCelcius = new double[] {kickerMotor.getMotorTemperature()};
  }

  @Override
  public void setHopperVoltage(double volts) {
    hopperMotor.setVoltage(volts);
  }

  @Override
  public void setHopperBrakeMode(boolean enable) {
    hopperMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTowerVoltage(double volts) {
    towerMotor.setVoltage(volts);
  }

  @Override
  public void setTowerBrakeMode(boolean enable) {
    towerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotor.setVoltage(volts);
  }

  @Override
  public void setKickerBrakeMode(boolean enable) {
    kickerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
