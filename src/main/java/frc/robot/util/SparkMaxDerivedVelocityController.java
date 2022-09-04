// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Alternative interface for reading the position and velocity of the internal encoder from a Spark
 * Max. This method uses the position data to calculate the velocity on the RIO, removing the ~100ms
 * latency from the default velocity measurement.
 */
public class SparkMaxDerivedVelocityController {
  private static final int deviceManufacturer = 5; // REV
  private static final int deviceType = 2; // Spark Max
  private static final int apiId = 98; // Periodic status 2

  private final CANSparkMax sparkMax;
  private final CAN canInterface;
  private final LinearFilter velocityFilter;
  private final PIDController velocityController;
  private final Notifier notifier;

  private boolean firstCycle = true;
  private boolean enabled = false;
  private double ffVolts = 0.0;
  private double timestamp = 0.0;
  private double position = 0.0;
  private double velocity = 0.0;

  /** Creates a new SparkMaxDerivedVelocityController. */
  public SparkMaxDerivedVelocityController(CANSparkMax sparkMax) {
    this(sparkMax, 0.005, 12);
  }

  /** Creates a new SparkMaxDerivedVelocityController. */
  public SparkMaxDerivedVelocityController(CANSparkMax sparkMax,
      double periodSeconds, int averagingTaps) {
    this.sparkMax = sparkMax;
    sparkMax.getEncoder().setPositionConversionFactor(1.0);
    int periodMs = (int) (periodSeconds * 1000);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, periodMs);

    canInterface =
        new CAN(sparkMax.getDeviceId(), deviceManufacturer, deviceType);
    velocityFilter = LinearFilter.movingAverage(averagingTaps);
    velocityController = new PIDController(0.0, 0.0, 0.0, periodSeconds);
    notifier = new Notifier(this::update);
    notifier.startPeriodic(periodSeconds);
  }

  /**
   * Reads new data, updates the velocity measurement, and runs the controller.
   */
  private void update() {
    CANData canData = new CANData();
    boolean isFresh = canInterface.readPacketNew(apiId, canData);
    double newTimestamp = canData.timestamp / 1000.0;
    double newPosition = ByteBuffer.wrap(canData.data)
        .order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);

    if (isFresh) {
      if (!firstCycle) {
        velocity = velocityFilter.calculate(
            (newPosition - position) / (newTimestamp - timestamp) * 60);
      }
      firstCycle = false;
      timestamp = newTimestamp;
      position = newPosition;

      if (enabled) {
        sparkMax.setVoltage(ffVolts + velocityController.calculate(velocity));
      }
    }
  }

  public synchronized void setReference(double velocityRpm, double ffVolts) {
    velocityController.setSetpoint(velocityRpm);
    this.ffVolts = ffVolts;

    if (!enabled) {
      velocityController.reset();
    }
    enabled = true;
  }

  public synchronized void disable() {
    sparkMax.stopMotor();
    enabled = false;
  }

  public synchronized void setPID(double kp, double ki, double kd) {
    velocityController.setPID(kp, ki, kd);
  }

  /**
   * Returns the encoder position in rotations.
   */
  public synchronized double getPosition() {
    return position;
  }

  /**
   * Returns the encoder velocity in rotations/minute.
   */
  public synchronized double getVelocity() {
    return velocity;
  }
}
