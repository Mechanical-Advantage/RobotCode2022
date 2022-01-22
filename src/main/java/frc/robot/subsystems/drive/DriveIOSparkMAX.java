package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveIOSparkMAX implements DriveIO {
  private final double afterEncoderReduction;

  private final boolean threeControllers;
  private final CANSparkMax leftLeader;
  private final CANSparkMax leftFollower;
  private CANSparkMax leftFollower2;
  private final CANSparkMax rightLeader;
  private final CANSparkMax rightFollower;
  private CANSparkMax rightFollower2;

  private final boolean externalEncoders;
  private Encoder leftExternalEncoder;
  private Encoder rightExternalEncoder;
  private RelativeEncoder leftInternalEncoder;
  private RelativeEncoder rightInternalEncoder;
  private final SparkMaxPIDController leftPID;
  private final SparkMaxPIDController rightPID;

  private final AHRS gyro = new AHRS(SPI.Port.kMXP); // SPI currently broken on 2022

  public DriveIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
        afterEncoderReduction = 1.0;
        threeControllers = true;
        leftLeader = new CANSparkMax(0, MotorType.kBrushless);
        leftFollower = new CANSparkMax(0, MotorType.kBrushless);
        leftFollower2 = new CANSparkMax(0, MotorType.kBrushless);
        rightLeader = new CANSparkMax(0, MotorType.kBrushless);
        rightFollower = new CANSparkMax(0, MotorType.kBrushless);
        rightFollower2 = new CANSparkMax(0, MotorType.kBrushless);

        externalEncoders = true;
        leftExternalEncoder = new Encoder(0, 1);
        rightExternalEncoder = new Encoder(2, 3);

        // Convert to rotations
        leftExternalEncoder.setDistancePerPulse(1440);
        rightExternalEncoder.setDistancePerPulse(1440);
        break;
      case ROBOT_2020:
        afterEncoderReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
        threeControllers = false;
        leftLeader = new CANSparkMax(3, MotorType.kBrushless);
        leftFollower = new CANSparkMax(12, MotorType.kBrushless);
        rightLeader = new CANSparkMax(16, MotorType.kBrushless);
        rightFollower = new CANSparkMax(15, MotorType.kBrushless);

        externalEncoders = false;
        leftInternalEncoder = leftLeader.getEncoder();
        rightInternalEncoder = rightLeader.getEncoder();
        break;
      default:
        throw new RuntimeException("Invalid robot for DriveIOSparkMax!");
    }

    leftPID = leftLeader.getPIDController();
    rightPID = rightLeader.getPIDController();

    if (Constants.burnMotorControllerFlash) {
      leftLeader.restoreFactoryDefaults();
      leftFollower.restoreFactoryDefaults();
      rightLeader.restoreFactoryDefaults();
      rightFollower.restoreFactoryDefaults();
      if (threeControllers) {
        leftFollower2.restoreFactoryDefaults();
        rightFollower2.restoreFactoryDefaults();
      }
    }

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    if (threeControllers) {
      leftFollower2.follow(leftLeader);
      rightFollower2.follow(rightLeader);
    }

    rightLeader.setInverted(false);
    leftLeader.setInverted(true);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);

    leftLeader.setSmartCurrentLimit(80);
    leftFollower.setSmartCurrentLimit(80);
    rightLeader.setSmartCurrentLimit(80);
    rightFollower.setSmartCurrentLimit(80);
    if (threeControllers) {
      leftFollower2.setSmartCurrentLimit(80);
      rightFollower2.setSmartCurrentLimit(80);
    }

    if (Constants.burnMotorControllerFlash) {
      leftLeader.burnFlash();
      leftFollower.burnFlash();
      rightLeader.burnFlash();
      rightFollower.burnFlash();
      if (threeControllers) {
        leftFollower2.burnFlash();
        rightFollower2.burnFlash();
      }
    }

    gyro.zeroYaw();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (externalEncoders) {
      inputs.leftPositionRad = leftExternalEncoder.getDistance()
          * (2.0 * Math.PI) / afterEncoderReduction;
      inputs.rightPositionRad = rightExternalEncoder.getDistance()
          * (2.0 * Math.PI) / afterEncoderReduction;

      inputs.leftVelocityRadPerSec = leftExternalEncoder.getRate()
          * (2.0 * Math.PI) / afterEncoderReduction;
      inputs.rightVelocityRadPerSec =
          rightExternalEncoder.getRate() * (2.0 * Math.PI)
              / afterEncoderReduction;
    } else {
      inputs.leftPositionRad = leftInternalEncoder.getPosition()
          * (2.0 * Math.PI) / afterEncoderReduction;
      inputs.rightPositionRad = rightInternalEncoder.getPosition()
          * (2.0 * Math.PI) / afterEncoderReduction;

      inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
          leftInternalEncoder.getVelocity()) / afterEncoderReduction;
      inputs.rightVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(
              rightInternalEncoder.getVelocity()) / afterEncoderReduction;
    }

    inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * 12.0;
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * 12.0;

    if (threeControllers) {
      inputs.leftCurrentAmps = new double[] {leftLeader.getOutputCurrent(),
          leftFollower.getOutputCurrent(), leftFollower2.getOutputCurrent()};
      inputs.rightCurrentAmps = new double[] {rightLeader.getOutputCurrent(),
          rightFollower.getOutputCurrent(), rightFollower2.getOutputCurrent()};

      inputs.leftTempCelcius = new double[] {leftLeader.getMotorTemperature(),
          leftFollower.getMotorTemperature(),
          leftFollower2.getMotorTemperature()};
      inputs.rightTempCelcius = new double[] {rightLeader.getMotorTemperature(),
          rightFollower.getMotorTemperature(),
          rightFollower2.getMotorTemperature()};
    } else {
      inputs.leftCurrentAmps = new double[] {leftLeader.getOutputCurrent(),
          leftFollower.getOutputCurrent()};
      inputs.rightCurrentAmps = new double[] {rightLeader.getOutputCurrent(),
          rightFollower.getOutputCurrent()};

      inputs.leftTempCelcius = new double[] {leftLeader.getMotorTemperature(),
          leftFollower.getMotorTemperature()};
      inputs.rightTempCelcius = new double[] {rightLeader.getMotorTemperature(),
          rightFollower.getMotorTemperature()};
    }

    inputs.gyroPositionRad = Math.toRadians(gyro.getAngle());
    inputs.gyroVelocityRadPerSec = Math.toRadians(gyro.getRate());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    double leftRPM =
        Units.radiansPerSecondToRotationsPerMinute(leftVelocityRadPerSec)
            * afterEncoderReduction;
    double rightRPM =
        Units.radiansPerSecondToRotationsPerMinute(rightVelocityRadPerSec)
            * afterEncoderReduction;
    leftPID.setReference(leftRPM, ControlType.kVelocity, 0, leftFFVolts,
        ArbFFUnits.kVoltage);
    rightPID.setReference(rightRPM, ControlType.kVelocity, 0, rightFFVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    leftLeader.setIdleMode(mode);
    leftFollower.setIdleMode(mode);
    rightLeader.setIdleMode(mode);
    rightFollower.setIdleMode(mode);
    if (threeControllers) {
      leftFollower2.setIdleMode(mode);
      rightFollower2.setIdleMode(mode);
    }
  }

  @Override
  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);

    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }
}
