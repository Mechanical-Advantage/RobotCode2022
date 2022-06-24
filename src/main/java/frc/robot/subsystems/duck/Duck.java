// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.duck.DuckIO.DuckIOInputs;

public class Duck extends SubsystemBase {
  private static final Map<DuckSound, Double> soundDurations = Map.of(
      DuckSound.MATCH_START, 1.49, DuckSound.QUACK_1, 0.22, DuckSound.QUACK_2,
      0.59, DuckSound.QUACK_3, 0.41, DuckSound.QUACK_4, 0.56,
      DuckSound.DONALD_ANGRY, 2.78, DuckSound.DONALD_COMING_THROUGH, 1.96);

  private final DuckIO io;
  private final DuckIOInputs inputs = new DuckIOInputs();

  /** Creates a new Duck. */
  public Duck(DuckIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Duck", inputs);
  }

  /** Run the duck at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  /** Stops the movement of the duck. */
  public void stop() {
    runPercent(0.0);
  }

  /** Plays the specified sound. */
  public void playSound(DuckSound sound) {
    io.playSound(sound.ordinal());
  }

  /** Gets the duration in seconds for a sound. */
  public static double getDuration(DuckSound sound) {
    return soundDurations.get(sound);
  }

  public static enum DuckSound {
    MATCH_START, QUACK_1, QUACK_2, QUACK_3, QUACK_4, DONALD_ANGRY, DONALD_COMING_THROUGH
  }
}
