// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class DerivedVelocityCalculator {
  private static final double maxCycleDurationPercentDiff = 0.05;
  private static final double maxVelocityPercentDiff = 0.25;

  private final TimeUnit timeUnit;

  private double velocity = 0.0;
  private double[] positionHistory = new double[] {0.0, 0.0, 0.0};
  private double[] timestampHistory = new double[] {0.0, 0.0, 0.0};

  public DerivedVelocityCalculator(TimeUnit timeUnit) {
    this.timeUnit = timeUnit;
  }

  public double calculate(double position) {
    positionHistory[2] = positionHistory[1];
    positionHistory[1] = positionHistory[0];
    positionHistory[0] = position;

    timestampHistory[2] = timestampHistory[1];
    timestampHistory[1] = timestampHistory[0];
    timestampHistory[0] = Timer.getFPGATimestamp();

    double firstDuration = timestampHistory[2] - timestampHistory[1];
    double secondDuration = timestampHistory[1] - timestampHistory[0];
    double cycleDurationPercentDiff =
        (secondDuration - firstDuration) / firstDuration;

    double firstVelocity = (positionHistory[1] - positionHistory[2])
        / timeUnit.fromSecs(timestampHistory[1] - timestampHistory[2]);
    double secondVelocity = (positionHistory[0] - positionHistory[1])
        / timeUnit.fromSecs(timestampHistory[0] - timestampHistory[1]);
    double velocityPercentDiff =
        (secondVelocity - firstVelocity) / firstVelocity;

    if (positionHistory[0] != positionHistory[1]
        && positionHistory[1] != positionHistory[2]) {
      if (Math.abs(cycleDurationPercentDiff) < maxCycleDurationPercentDiff
          && Math.abs(velocityPercentDiff) < maxVelocityPercentDiff) {
        velocity = firstVelocity;
      }
    } else if (positionHistory[0] == positionHistory[1]
        && positionHistory[1] == positionHistory[2]) {
      velocity = 0.0;
    }

    return velocity;
  }

  public static enum TimeUnit {
    SECOND, MINUTE, HOUR;

    private double fromSecs(double secs) {
      switch (this) {
        case SECOND:
          return secs;
        case MINUTE:
          return secs / 60.0;
        case HOUR:
          return secs / (60.0 * 60.0);
        default:
          return secs;
      }
    }
  }
}
