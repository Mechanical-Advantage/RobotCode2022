package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.romi.RomiMotor;

public class DriveIORomi implements DriveIO {

  private final RomiMotor leftMotor = new RomiMotor(0);
  private final RomiMotor rightMotor = new RomiMotor(1);
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);
  private final RomiGyro gyro = new RomiGyro();
  private PIDController leftPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController rightPID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
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
    if (closedLoop) {
      double leftVolts = leftPID.calculate(leftEncoder.getRate()) + leftFFVolts;
      double rightVolts =
          rightPID.calculate(rightEncoder.getRate()) + rightFFVolts;
      appliedVoltsRight = rightVolts;
      appliedVoltsLeft = leftVolts;
      leftMotor.setVoltage(leftVolts);
      rightMotor.setVoltage(rightVolts);
    }

    inputs.leftPositionRad = leftEncoder.getDistance();
    inputs.leftAppliedVolts = appliedVoltsLeft;
    inputs.leftCurrentAmps = new double[] {};
    inputs.leftTempCelcius = new double[] {};

    inputs.rightPositionRad = rightEncoder.getDistance();
    inputs.rightCurrentAmps = new double[] {};
    inputs.rightAppliedVolts = appliedVoltsRight;
    inputs.rightTempCelcius = new double[] {};

    inputs.gyroPositionRad = Math.toRadians(gyro.getAngleZ());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
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
