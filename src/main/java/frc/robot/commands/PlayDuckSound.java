// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.duck.Duck;
import frc.robot.subsystems.duck.Duck.DuckSound;

public class PlayDuckSound extends WaitCommand {
  private static final double duckPercentage = 0.75;

  private final Duck duck;
  private final DuckSound sound;

  /** Creates a new PlayDuckSound. Plays the specified duck sound while spinning the duck. */
  public PlayDuckSound(Duck duck, DuckSound sound) {
    super(Duck.soundLengths.get(sound));
    addRequirements(duck);
    this.duck = duck;
    this.sound = sound;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    duck.runPercent(duckPercentage);
    duck.playSound(sound);
  }
}
