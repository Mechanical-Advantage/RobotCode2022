package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import frc.robot.Constants;

public class DriveIORomi implements DriveIO {

  private boolean closedLoop = false;
  private final RomiGyro gyro = new RomiGyro();
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);
  private PIDController leftPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController rightPID = new PIDController(0.0, 0.0, 0.0);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private double appliedVoltsLeft = 0.0;
  private double appliedVoltsRight = 0.0;


  public DriveIORomi() {
    // Romi encoders have 1440 pulses per revolution
    leftEncoder.setDistancePerPulse((2 * Math.PI) / 1440);
    rightEncoder.setDistancePerPulse((2 * Math.PI) / 1440);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {

    inputs.gyroPositionRad = Math.toRadians(gyro.getAngleZ());
    inputs.rightPositionRad = (rightEncoder.getDistance() / 1440) * 2 * Math.PI;
    inputs.leftPositionRad = (leftEncoder.getDistance() / 1440) * 2 * Math.PI;
    inputs.leftCurrentAmps = new double[] {};
    inputs.rightCurrentAmps = new double[] {};
    inputs.leftTempCelcius = new double[] {};
    inputs.rightTempCelcius = new double[] {};
    inputs.leftAppliedVolts = appliedVoltsLeft;
    inputs.rightAppliedVolts = appliedVoltsRight;
    double leftVolts = leftPID.calculate(leftEncoder.getRate()) + leftFFVolts;
    double rightVolts =
        rightPID.calculate(rightEncoder.getRate()) + rightFFVolts;
    setVoltage(leftVolts, rightVolts);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    appliedVoltsRight = rightVolts;
    appliedVoltsLeft = leftVolts;
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);

  }

  @Override
  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {

    closedLoop = true;
    leftPID.setSetpoint(leftVelocityRadPerSec);
    rightPID.setSetpoint(rightVelocityRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;

  }

  @Override
  public void configurePID(double kp, double ki, double kd) {

  }

  @Override
  public void resetPosition(double leftPositionRad, double rightPositionRad) {
    leftEncoder.reset();
    rightEncoder.reset();
  }
}
