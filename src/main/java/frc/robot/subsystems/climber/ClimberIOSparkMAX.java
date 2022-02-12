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
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
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
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        leader = new CANSparkMax(0, MotorType.kBrushless);
        follower = new CANSparkMax(0, MotorType.kBrushless);
        invert = false;
        invertFollower = false;
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

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);

    if (SparkMAXBurnManager.shouldBurn()) {
      leader.burnFlash();
      follower.burnFlash();
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.locked = !solenoid.get();
    inputs.positionRad =
        encoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.appliedVolts = leader.getAppliedOutput() * 12.0;
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
  public void setLocked(boolean locked) {
    solenoid.set(!locked);
  }
}
