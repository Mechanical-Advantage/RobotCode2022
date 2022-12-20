// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class DriveIOTalonSRX implements DriveIO {
  private final TalonSRX leftLeader;
  private final TalonSRX leftFollower;
  private final TalonSRX rightLeader;
  private final TalonSRX rightFollower;

  private final double encoderTicksPerRev;

  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  public DriveIOTalonSRX() {
    switch (Constants.getRobot()) {
      case ROBOT_KITBOT:
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.peakCurrentLimit = 40;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

        leftLeader = new TalonSRX(1);
        leftFollower = new TalonSRX(2);
        rightLeader = new TalonSRX(3);
        rightFollower = new TalonSRX(4);
        encoderTicksPerRev = 1440.0;

        leftLeader.configAllSettings(config);
        rightLeader.configAllSettings(config);
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftLeader.setInverted(false);
        rightLeader.setInverted(true);
        leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.setInverted(InvertType.FollowMaster);
        break;
      default:
        throw new RuntimeException("Invalid robot for DriveIOTalonSRX!");
    }

    gyro.calibrate();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad =
        Units.rotationsToRadians(leftLeader.getSelectedSensorPosition() / encoderTicksPerRev);
    inputs.rightPositionRad =
        Units.rotationsToRadians(rightLeader.getSelectedSensorPosition() / encoderTicksPerRev);
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(leftLeader.getSelectedSensorVelocity() * 10 / encoderTicksPerRev);
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(rightLeader.getSelectedSensorVelocity() * 10 / encoderTicksPerRev);
    inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftLeader.getSupplyCurrent(), leftFollower.getSupplyCurrent()};
    inputs.rightCurrentAmps =
        new double[] {rightLeader.getSupplyCurrent(), rightFollower.getSupplyCurrent()};

    inputs.gyroConnected = gyro.isConnected();
    inputs.gyroYawPositionRad = Math.toRadians(gyro.getAngle());
    inputs.gyroYawVelocityRadPerSec = Math.toRadians(gyro.getRate());
    inputs.gyroPitchPositionRad = Math.toRadians(gyro.getRoll());
    inputs.gyroRollPositionRad = Math.toRadians(gyro.getPitch());
    inputs.gyroZAccelMetersPerSec2 = gyro.getWorldLinearAccelZ() * 9.806;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.set(ControlMode.PercentOutput, leftVolts / 12.0);
    rightLeader.set(ControlMode.PercentOutput, rightVolts / 12.0);
  }

  @Override
  public void setVelocity(
      double leftVelocityRadPerSec,
      double rightVelocityRadPerSec,
      double leftFFVolts,
      double rightFFVolts) {
    double leftTicksPer100Ms =
        Units.radiansToRotations(leftVelocityRadPerSec) * encoderTicksPerRev / 10.0;
    double rightTicksPer100Ms =
        Units.radiansToRotations(rightVelocityRadPerSec) * encoderTicksPerRev / 10.0;
    leftLeader.set(
        ControlMode.Velocity,
        leftTicksPer100Ms,
        DemandType.ArbitraryFeedForward,
        leftFFVolts / 12.0);
    rightLeader.set(
        ControlMode.Velocity,
        rightTicksPer100Ms,
        DemandType.ArbitraryFeedForward,
        rightFFVolts / 12.0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
    leftLeader.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
    rightLeader.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
  }

  @Override
  public void configurePID(double kp, double ki, double kd) {
    leftLeader.config_kP(0, kp);
    leftLeader.config_kI(0, ki);
    leftLeader.config_kD(0, kd);
    rightLeader.config_kP(0, kp);
    rightLeader.config_kI(0, ki);
    rightLeader.config_kD(0, kd);
  }
}
