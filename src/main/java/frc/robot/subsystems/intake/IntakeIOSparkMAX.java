// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class IntakeIOSparkMAX implements IntakeIO {
  private boolean invert = false;

  private double afterEncoderReduction = 1.0;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  // private final Solenoid solenoid;

  public IntakeIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022P:
        afterEncoderReduction = 1.0;
        // solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        motor = new CANSparkMax(4, MotorType.kBrushless);
        invert = true;
        break;
      default:
        throw new RuntimeException("Invalid robot for IntakeIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }
    motor.setInverted(invert);
    motor.setSmartCurrentLimit(30);
    motor.enableVoltageCompensation(12.0);

    encoder = motor.getEncoder();

    motor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.extended = solenoid.get();
    inputs.positionRad =
        encoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.appliedVolts = motor.getAppliedOutput();
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
    // solenoid.set(extended);
  }
}
