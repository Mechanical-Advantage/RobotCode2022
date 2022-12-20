// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.hood;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class HoodIOSparkMAX implements HoodIO {
  private boolean invert = false;
  private double afterEncoderReduction = 1.0;

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public HoodIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        motor = new CANSparkMax(3, MotorType.kBrushless);
        invert = true;
        // 4:1 UP stage -> 3:1 UP stage -> 3:1 UP stage -> 18:44 stage -> 18 tooth : 66 tooth pulley
        // UP stages are not actually integer, so this accounts for that:
        // 4:1 is actually 76:21, 3:1 is actually 84:29
        afterEncoderReduction =
            (76.0 / 21.0) * (84.0 / 29.0) * (84.0 / 29.0) * (44.0 / 18.0) * (66.0 / 18.0);
        break;
      default:
        throw new RuntimeException("Invalid robot for HoodIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(30);
    motor.enableVoltageCompensation(12.0);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

    encoder = motor.getEncoder();

    motor.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
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
}
