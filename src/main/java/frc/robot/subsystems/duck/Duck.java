// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.duck;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.duck.DuckIO.DuckIOInputs;

public class Duck extends SubsystemBase {

  // https://learn.adafruit.com/adafruit-audio-fx-sound-board/triggering-audio
  private static final double playDelaySecs = 0.12; // How long after pulse before sound starts
  private static final double pulseLengthSecs = 0.2; // Pulse length to trigger playback

  private static final Map<DuckSound, Double> soundLengthsSecs =
      Map.of(DuckSound.MATCH_START, 1.49, DuckSound.QUACK_1, 0.22,
          DuckSound.QUACK_2, 0.59, DuckSound.QUACK_3, 0.41, DuckSound.QUACK_4,
          0.56, DuckSound.QUACK_5, 2.78);

  private final DuckIO io;
  private final DuckIOInputs inputs = new DuckIOInputs();

  private boolean pulseActive = false;
  private Timer pulseTimer = new Timer();

  /** Creates a new Duck. */
  public Duck(DuckIO io) {
    this.io = io;

    io.setActive(null);
    pulseActive = false;
    pulseTimer.reset();
    pulseTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Duck", inputs);

    if (pulseActive) {
      if (pulseTimer.hasElapsed(pulseLengthSecs)) {
        io.setActive(null);
        pulseActive = false;
      }
    }
  }

  /** Run the duck at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  /** Plays the specified sound. */
  public void playSound(DuckSound sound) {
    io.setActive(sound);
    pulseActive = true;
    pulseTimer.reset();
  }

  /** Gets the duration from trigger until a sound's completion. */
  public static double getDuration(DuckSound sound) {
    return playDelaySecs + soundLengthsSecs.get(sound);
  }

  public static enum DuckSound {
    MATCH_START, QUACK_1, QUACK_2, QUACK_3, QUACK_4, QUACK_5
  }
}
