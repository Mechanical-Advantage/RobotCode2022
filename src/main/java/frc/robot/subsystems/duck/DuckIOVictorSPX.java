// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.util.SparkMAXBurnManager;

public class DuckIOVictorSPX implements DuckIO {
  private boolean invert = false;

  private double afterEncoderReduction = 1.0;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final DoubleSolenoid solenoid;

  public DuckIOVictorSPX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        solenoid = new DoubleSolenoid(Pneumatics.revModuleID,
            PneumaticsModuleType.REVPH, 2, 5);
        motor = new CANSparkMax(4, MotorType.kBrushless);
        afterEncoderReduction = 60.0 / 16.0;
        invert = true;

        encoder = motor.getEncoder();
        break;
      default:
        throw new RuntimeException("Invalid robot for DuckIOVictorSPX!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(30);
    motor.enableVoltageCompensation(12.0);

    motor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(DuckIOInputs inputs) {
    inputs.extended = solenoid.get() == Value.kReverse;

    inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition()) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            / afterEncoderReduction;
    inputs.appliedVolts =
        motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setExtended(boolean extended) {
    solenoid.set(extended ? Value.kReverse : Value.kForward);
  }
}
