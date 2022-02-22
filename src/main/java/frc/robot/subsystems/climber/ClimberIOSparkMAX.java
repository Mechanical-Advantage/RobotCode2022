// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.util.SparkMAXBurnManager;

public class ClimberIOSparkMAX implements ClimberIO {
  private boolean invert = false;
  private boolean invertFollower = false;
  private double afterEncoderReduction = 1.0;

  private final Solenoid solenoid;

  private final CANSparkMax leader;
  private final CANSparkMax follower;
  private final RelativeEncoder encoder;

  public ClimberIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        solenoid =
            new Solenoid(Pneumatics.revModuleID, PneumaticsModuleType.REVPH, 0);
        leader = new CANSparkMax(6, MotorType.kBrushless);
        follower = new CANSparkMax(14, MotorType.kBrushless);
        invert = false;
        invertFollower = true;
        afterEncoderReduction = 20.0 * (50.0 / 20.0);
        break;
      default:
        throw new RuntimeException("Invalid robot for ClimberIOSparkMax!");
    }

    if (SparkMAXBurnManager.shouldBurn()) {
      leader.restoreFactoryDefaults();
      follower.restoreFactoryDefaults();
    }

    follower.follow(leader, invertFollower);

    leader.setInverted(invert);
    leader.setSmartCurrentLimit(30);
    follower.setSmartCurrentLimit(30);
    leader.enableVoltageCompensation(12.0);
    follower.enableVoltageCompensation(12.0);

    encoder = leader.getEncoder();
    encoder.setPosition(0.0);

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      leader.burnFlash();
      follower.burnFlash();
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.unlocked = solenoid.get();
    inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition()) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            / afterEncoderReduction;
    inputs.appliedVolts =
        leader.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
    inputs.tempCelcius = new double[] {leader.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setUnlocked(boolean unlocked) {
    solenoid.set(unlocked);
  }
}
