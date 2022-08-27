// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMaxDerivedVelocityController;

public class TestFlywheels extends SubsystemBase {
  private static final double afterEncoderReduction = (48.0 / 24.0);
  private static final boolean invert = true;

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxDerivedVelocityController derivedVelocityController;

  /** Creates a new TestFlywheels. */
  public TestFlywheels() {
    motor = new CANSparkMax(2, MotorType.kBrushless);

    motor.restoreFactoryDefaults();

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(50);
    motor.enableVoltageCompensation(12.0);

    encoder = motor.getEncoder();
    derivedVelocityController =
        new SparkMaxDerivedVelocityController(motor, 0.01);
    motor.setCANTimeout(0);
    motor.burnFlash();
  }

  @Override
  public void periodic() {
    Logger.getInstance().recordOutput("TestFlywheels/PositionRotations",
        encoder.getPosition());
    Logger.getInstance().recordOutput("TestFlywheels/SimpleRPM",
        encoder.getVelocity() / afterEncoderReduction);
    Logger.getInstance().recordOutput("TestFlywheels/DerivedRPM",
        derivedVelocityController.getVelocity() / afterEncoderReduction);
  }

  public void runVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
