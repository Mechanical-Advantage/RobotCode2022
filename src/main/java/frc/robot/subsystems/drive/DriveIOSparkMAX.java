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
import frc.robot.util.SparkMAXBurnManager;

public class DriveIOSparkMAX implements DriveIO {

  private final boolean hasThreeControllers;
  private final boolean leftInverted;
  private final boolean rightInverted;
  private final CANSparkMax leftLeader;
  private final CANSparkMax leftFollower;
  private CANSparkMax leftFollower2;
  private final CANSparkMax rightLeader;
  private final CANSparkMax rightFollower;
  private CANSparkMax rightFollower2;

  private final double afterEncoderReduction;
  private final boolean hasExternalEncoders;
  private Encoder leftExternalEncoder;
  private Encoder rightExternalEncoder;
  private RelativeEncoder leftInternalEncoder;
  private RelativeEncoder rightInternalEncoder;
  private final SparkMaxPIDController leftPID;
  private final SparkMaxPIDController rightPID;

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public DriveIOSparkMAX() {
    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        afterEncoderReduction = 6.0; // Internal encoders
        hasExternalEncoders = false;
        hasThreeControllers = true;
        leftInverted = true;
        rightInverted = false;

        leftLeader = new CANSparkMax(10, MotorType.kBrushless);
        leftFollower = new CANSparkMax(11, MotorType.kBrushless);
        leftFollower2 = new CANSparkMax(12, MotorType.kBrushless);
        rightLeader = new CANSparkMax(7, MotorType.kBrushless);
        rightFollower = new CANSparkMax(8, MotorType.kBrushless);
        rightFollower2 = new CANSparkMax(9, MotorType.kBrushless);
        break;
      case ROBOT_2022P:
        afterEncoderReduction = 6.0; // Internal encoders
        hasExternalEncoders = true;
        hasThreeControllers = true;
        leftInverted = true;
        rightInverted = false;

        leftLeader = new CANSparkMax(12, MotorType.kBrushless);
        leftFollower = new CANSparkMax(2, MotorType.kBrushless);
        leftFollower2 = new CANSparkMax(3, MotorType.kBrushless);
        rightLeader = new CANSparkMax(15, MotorType.kBrushless);
        rightFollower = new CANSparkMax(1, MotorType.kBrushless);
        rightFollower2 = new CANSparkMax(30, MotorType.kBrushless);

        leftExternalEncoder = new Encoder(2, 3);
        rightExternalEncoder = new Encoder(0, 1);
        leftExternalEncoder.setDistancePerPulse(-1.0 / 2048.0);
        rightExternalEncoder.setDistancePerPulse(1.0 / 2048.0);
        break;
      case ROBOT_2020:
        afterEncoderReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
        hasExternalEncoders = false;
        hasThreeControllers = false;
        leftInverted = true;
        rightInverted = false;

        leftLeader = new CANSparkMax(3, MotorType.kBrushless);
        leftFollower = new CANSparkMax(12, MotorType.kBrushless);
        rightLeader = new CANSparkMax(16, MotorType.kBrushless);
        rightFollower = new CANSparkMax(15, MotorType.kBrushless);
        break;
      default:
        throw new RuntimeException("Invalid robot for DriveIOSparkMax!");
    }

    leftPID = leftLeader.getPIDController();
    rightPID = rightLeader.getPIDController();

    leftInternalEncoder = leftLeader.getEncoder();
    rightInternalEncoder = rightLeader.getEncoder();
    leftInternalEncoder.setPosition(0.0);
    rightInternalEncoder.setPosition(0.0);

    if (SparkMAXBurnManager.shouldBurn()) {
      leftLeader.restoreFactoryDefaults();
      leftFollower.restoreFactoryDefaults();
      rightLeader.restoreFactoryDefaults();
      rightFollower.restoreFactoryDefaults();
      if (hasThreeControllers) {
        leftFollower2.restoreFactoryDefaults();
        rightFollower2.restoreFactoryDefaults();
      }
    }

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    if (hasThreeControllers) {
      leftFollower2.follow(leftLeader);
      rightFollower2.follow(rightLeader);
    }

    leftLeader.setInverted(leftInverted);
    rightLeader.setInverted(rightInverted);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);

    leftLeader.setSmartCurrentLimit(30);
    leftFollower.setSmartCurrentLimit(30);
    rightLeader.setSmartCurrentLimit(30);
    rightFollower.setSmartCurrentLimit(30);
    if (hasThreeControllers) {
      leftFollower2.setSmartCurrentLimit(30);
      rightFollower2.setSmartCurrentLimit(30);
    }

    leftLeader.setCANTimeout(0);
    leftFollower.setCANTimeout(0);
    rightLeader.setCANTimeout(0);
    rightFollower.setCANTimeout(0);
    if (hasThreeControllers) {
      leftFollower2.setCANTimeout(0);
      rightFollower2.setCANTimeout(0);
    }


    if (SparkMAXBurnManager.shouldBurn()) {
      leftLeader.burnFlash();
      leftFollower.burnFlash();
      rightLeader.burnFlash();
      rightFollower.burnFlash();
      if (hasThreeControllers) {
        leftFollower2.burnFlash();
        rightFollower2.burnFlash();
      }
    }

    gyro.zeroYaw();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad =
        Units.rotationsToRadians(leftInternalEncoder.getPosition())
            / afterEncoderReduction;
    inputs.rightPositionRad =
        Units.rotationsToRadians(rightInternalEncoder.getPosition())
            / afterEncoderReduction;
    inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        leftInternalEncoder.getVelocity()) / afterEncoderReduction;
    inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        rightInternalEncoder.getVelocity()) / afterEncoderReduction;

    inputs.externalAvailable = hasExternalEncoders;
    if (hasExternalEncoders) {
      inputs.externalLeftPositionRad =
          Units.rotationsToRadians(leftExternalEncoder.getDistance());
      inputs.externalRightPositionRad =
          Units.rotationsToRadians(rightExternalEncoder.getDistance());
      inputs.externalLeftVelocityRadPerSec =
          Units.rotationsToRadians(leftExternalEncoder.getRate());
      inputs.externalRightVelocityRadPerSec =
          Units.rotationsToRadians(rightExternalEncoder.getRate());
    }

    inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * 12.0;
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * 12.0;

    if (hasThreeControllers) {
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
    if (hasThreeControllers) {
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
