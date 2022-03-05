// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.TunableNumber;

public class ResetClimber extends CommandBase {
  private static final TunableNumber startDelaySecs =
      new TunableNumber("ResetClimber/StartDelaySecs");
  private static final TunableNumber startSpeedPercent =
      new TunableNumber("ResetClimber/StartSpeed");
  private static final TunableNumber normalSpeedPercent =
      new TunableNumber("ResetClimber/NormalSpeed");
  private static final TunableNumber graceSecs =
      new TunableNumber("ResetClimber/GraceSecs");
  private static final TunableNumber backSecs =
      new TunableNumber("ResetClimber/BackSecs");
  private static final TunableNumber currentThreshold =
      new TunableNumber("ResetClimber/CurrentThreshhold");

  private final Climber climber;
  private final Timer timer = new Timer();
  private boolean started = false;
  private boolean backwards = false;

  /** Creates a new ResetClimber. */
  public ResetClimber(Climber climber) {
    addRequirements(climber);
    this.climber = climber;

    startDelaySecs.setDefault(0.4);
    startSpeedPercent.setDefault(0.05);
    normalSpeedPercent.setDefault(0.1);
    graceSecs.setDefault(0.1);
    backSecs.setDefault(0.1);
    currentThreshold.setDefault(40.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    started = false;
    backwards = false;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetClimber", true);

    if (!started) {
      climber.runPercent(startSpeedPercent.get());
      if (timer.hasElapsed(startDelaySecs.get())) {
        started = true;
      }
    } else if (!backwards) {
      climber.runPercent(-normalSpeedPercent.get());
      if (timer.hasElapsed(graceSecs.get())) {
        climber.runPercent(-normalSpeedPercent.get());
        if (climber.getCurrentAmps() > currentThreshold.get()) {
          backwards = true;
          timer.reset();
          climber.resetPosition();
        }
      }
    } else {
      climber.runPercent(normalSpeedPercent.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runPercent(0.0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return started && backwards && timer.hasElapsed(backSecs.get());
  }
}
