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
  private static final TunableNumber speedPercent =
      new TunableNumber("ResetClimber/Speed");
  private static final TunableNumber graceSecs =
      new TunableNumber("ResetClimber/GraceSecs");
  private static final TunableNumber backSecs =
      new TunableNumber("ResetClimber/BackSecs");
  private static final TunableNumber currentThreshold =
      new TunableNumber("ResetClimber/CurrentThreshhold");

  private final Climber climber;
  private final Timer timer = new Timer();
  private boolean backwards = false;

  /** Creates a new ResetClimber. */
  public ResetClimber(Climber climber) {
    addRequirements(climber);
    this.climber = climber;

    speedPercent.setDefault(0.2);
    graceSecs.setDefault(0.2);
    backSecs.setDefault(0.2);
    currentThreshold.setDefault(15.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetClimber", true);

    if (backwards) {
      climber.runPercent(speedPercent.get());
    } else {
      climber.runPercent(-speedPercent.get());
      if (timer.hasElapsed(graceSecs.get())) {
        climber.runPercent(-speedPercent.get());
        if (climber.getCurrentAmps() > currentThreshold.get()) {
          backwards = true;
          timer.reset();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runPercent(0.0);
    climber.resetPosition();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return backwards && timer.hasElapsed(backSecs.get());
  }
}
