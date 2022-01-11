package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class DriveIOSparkMAX implements DriveIO {
  private static final double afterEncoderReduction =
      1.0 / ((9.0 / 62.0) * (18.0 / 30.0));



  private final CANSparkMax leftLeader =
      new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax leftFollower =
      new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax rightLeader =
      new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax rightFollower =
      new CANSparkMax(15, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final SparkMaxPIDController leftPID = leftLeader.getPIDController();
  private final SparkMaxPIDController rightPID = rightLeader.getPIDController();


  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // SPI currently broken on 2022


  public DriveIOSparkMAX() {
    if (Constants.burnMotorControllerFlash) {
      leftLeader.restoreFactoryDefaults();
      leftFollower.restoreFactoryDefaults();
      rightLeader.restoreFactoryDefaults();
      rightFollower.restoreFactoryDefaults();
    }

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    rightLeader.setInverted(false);
    leftLeader.setInverted(true);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);

    leftLeader.setSmartCurrentLimit(80);
    leftFollower.setSmartCurrentLimit(80);
    rightLeader.setSmartCurrentLimit(80);
    rightFollower.setSmartCurrentLimit(80);

    setBrakeMode(false);

    if (Constants.burnMotorControllerFlash) {
      leftLeader.burnFlash();
      leftFollower.burnFlash();
      rightLeader.burnFlash();
      rightFollower.burnFlash();
    }

    gyro.zeroYaw();
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

  @Override
  public void resetPosition(double leftPositionRad, double rightPositionRad) {
    leftEncoder
        .setPosition(leftPositionRad / (2.0 * Math.PI) * afterEncoderReduction);
    rightEncoder.setPosition(
        rightPositionRad / (2.0 * Math.PI) * afterEncoderReduction);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    leftLeader.setIdleMode(mode);
    leftFollower.setIdleMode(mode);
    rightLeader.setIdleMode(mode);
    rightFollower.setIdleMode(mode);
  }

  @Override
  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    double leftRPM =
        leftVelocityRadPerSec * 60.0 / (2.0 * Math.PI) * afterEncoderReduction;
    double rightRPM =
        leftVelocityRadPerSec * 60.0 / (2.0 * Math.PI) * afterEncoderReduction;
    leftPID.setReference(leftRPM, ControlType.kVelocity, 0, leftFFVolts,
        ArbFFUnits.kVoltage);
    rightPID.setReference(rightRPM, ControlType.kVelocity, 0, rightFFVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void updateInputs(DriveTrainIOInputs inputs) {
    inputs.leftPositionRad =
        leftEncoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;
    inputs.rightPositionRad =
        rightEncoder.getPosition() * (2.0 * Math.PI) / afterEncoderReduction;

    inputs.leftVelocityRadPerSec = leftEncoder.getVelocity() * (2.0 * Math.PI)
        / 60.0 / afterEncoderReduction;
    inputs.rightVelocityRadPerSec = rightEncoder.getVelocity() * (2.0 * Math.PI)
        / 60.0 / afterEncoderReduction;

    inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * 12.0;
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * 12.0;

    inputs.leftCurrentAmps = new double[] {leftLeader.getOutputCurrent(),
        leftFollower.getOutputCurrent()};

    inputs.rightCurrentAmps = new double[] {rightLeader.getOutputCurrent(),
        rightFollower.getOutputCurrent()};

    inputs.leftTempCelcius = new double[] {leftLeader.getMotorTemperature(),
        leftFollower.getMotorTemperature()};
    inputs.rightTempCelcius = new double[] {rightLeader.getMotorTemperature(),
        rightFollower.getMotorTemperature()};

    inputs.gyroPositionRad = Math.toRadians(gyro.getAngle());
    inputs.gyroVelocityRadPerSec = Math.toRadians(gyro.getRate());

  }

}
