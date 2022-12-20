// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.util.SparkMAXBurnManager;

public class IntakeIOSparkMAX implements IntakeIO {
  private boolean invert = false;

  private double afterEncoderReduction = 1.0;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final DoubleSolenoid solenoid;

  public IntakeIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        solenoid = new DoubleSolenoid(Pneumatics.revModuleID, PneumaticsModuleType.REVPH, 2, 5);
        motor = new CANSparkMax(4, MotorType.kBrushless);
        afterEncoderReduction = 60.0 / 16.0;
        invert = true;

        encoder = motor.getEncoder();
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

    motor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extended = solenoid.get() == Value.kReverse;

    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / afterEncoderReduction;
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
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
