// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

/**
 * Alternative interface for reading the position and velocity of the internal encoder from a Spark
 * Max. This method uses the position data to calculate the velocity on the RIO, removing the ~100ms
 * latency from the default velocity measurement.
 * 
 * <p>
 * The {@link #update()} method should be called at the start of each periodic cycle, then
 * {@link #getPosition()} and {@link #getVelocity()} can be used to read the current values. Note
 * that the REVLib calls to read the position and velocity from the internal encoder will be
 * nonfunctional. All other functions of REVLib are unaffected.
 * 
 * <p>
 * We recommend leaving the position conversion factor on the Spark Max at the default value (1).
 * This will return units of rotations and rotations/minute, which can subsequently be converted to
 * your desired units.
 */
public class SparkMaxDerivedEncoder {
  private static final int deviceManufacturer = 5; // REV
  private static final int deviceType = 2; // Spark Max
  private static final int apiId = 98; // Periodic status 2

  private final CAN canInterface;
  private double timestamp = 0.0;
  private double position = 0.0;
  private double velocity = 0.0;

  /** Creates a new SparkMaxDerivedEncoder. */
  public SparkMaxDerivedEncoder(CANSparkMax sparkMax) {
    this(sparkMax.getDeviceId());
  }

  /** Creates a new SparkMaxDerivedEncoder. */
  public SparkMaxDerivedEncoder(int deviceId) {
    canInterface = new CAN(deviceId, deviceManufacturer, deviceType);
  }

  /**
   * Reads new data and updates the velocity measurement.
   */
  public void update() {
    CANData canData = new CANData();
    boolean isFresh = canInterface.readPacketNew(apiId, canData);
    double newTimestamp = canData.timestamp / 1e6;
    double newPosition = ByteBuffer.wrap(canData.data)
        .order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);

    if (isFresh) {
      velocity = (newPosition - position) / (newTimestamp - timestamp) * 60;
      timestamp = newTimestamp;
      position = newPosition;
    }
  }

  /**
   * Returns the encoder position in rotations.
   */
  public double getPosition() {
    return position;
  }

  /**
   * Returns the encoder velocity in rotations/minute.
   */
  public double getVelocity() {
    return velocity;
  }
}
