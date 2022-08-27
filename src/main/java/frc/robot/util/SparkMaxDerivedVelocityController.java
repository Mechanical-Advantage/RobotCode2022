// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;

public class SparkMaxDerivedVelocityController {
  private final ControllerThread thread;

  public SparkMaxDerivedVelocityController(CANSparkMax device, double period) {
    int periodMs = (int) (period * 1000);
    device.setControlFramePeriodMs(periodMs);
    device.setPeriodicFramePeriod(PeriodicFrame.kStatus2, periodMs);

    thread = new ControllerThread(device, period);
    thread.start();
  }

  public void setReference(double setpoint) {
    thread.setReference(setpoint);
  }

  public void disable() {
    thread.disable();
  }

  public void setPID(double kp, double ki, double kd) {
    thread.setPID(kp, ki, kd);
  }

  public double getVelocity() {
    return thread.getVelocity();
  }

  private static class ControllerThread extends Thread {
    private static final double sleepLengthMultiplier = 0.1;
    private static final double stationaryTimeoutMultiplier = 3.0;

    private final CANSparkMax device;
    private final double period;

    private final RelativeEncoder encoder;
    private final PIDController pid;
    private boolean enabled = false;
    private double velocity = 0.0;

    ControllerThread(CANSparkMax device, double period) {
      super("SparkMaxDerivedVelocityController_"
          + Integer.toString(device.getDeviceId()));
      this.device = device;
      this.period = period;
      encoder = device.getEncoder();
      pid = new PIDController(0.0, 0.0, 0.0, period);
    }

    public void run() {
      try {
        double lastTimestamp = Logger.getInstance().getRealTimestamp();
        double lastPosition = 0.0;
        boolean stationaryMode = false; // When stationary, run based on period instead of waiting
                                        // for an update. Use the full period for the first
                                        // non-stationary cycle.
        while (true) {
          Thread.sleep((long) (1000 * sleepLengthMultiplier * period));
          double timestamp = Logger.getInstance().getRealTimestamp();
          double position = encoder.getPosition();

          double timestampDelta = timestamp - lastTimestamp;
          if (stationaryMode) {
            if (position != lastPosition) { // Exit stationary mode
              System.out.println("Exiting stationary mode");
              stationaryMode = false;
              timestampDelta = period; // Assume a full period
            } else if (timestamp - lastTimestamp < period) { // Not a full period yet
              continue;
            }
          } else {
            if (timestamp - lastTimestamp > period
                * stationaryTimeoutMultiplier) { // Switch to stationary mode
              System.out.println("Entering stationary mode");
              stationaryMode = true;
            } else if (position == lastPosition) { // Not changed, wait for new data
              continue;
            }
          }

          synchronized (this) {
            velocity = ((position - lastPosition) / timestampDelta) * 60.0;
            if (enabled) {
              double output = pid.calculate(velocity);
              device.set(output);
            }
          }

          lastPosition = position;
          lastTimestamp = timestamp;
        }
      } catch (InterruptedException exception) {
        exception.printStackTrace();
      }
    }

    synchronized void setReference(double setpoint) {
      enabled = true;
      pid.setSetpoint(setpoint);
    }

    synchronized void disable() {
      enabled = false;
      device.stopMotor();
      pid.setSetpoint(0.0);
      pid.reset();
    }

    synchronized void setPID(double kp, double ki, double kd) {
      pid.setPID(kp, ki, kd);
    }

    synchronized double getVelocity() {
      return velocity;
    }
  }
}
