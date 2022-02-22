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
      new TunableNumber("ResetClimber/Grace");
  private static final TunableNumber velocityThresholdRadPerSec =
      new TunableNumber("ResetClimber/VelocityThreshhold");

  private final Climber climber;
  private final Timer graceTimer = new Timer();

  /** Creates a new ResetClimber. */
  public ResetClimber(Climber climber) {
    addRequirements(climber);
    this.climber = climber;

    speedPercent.setDefault(-0.1);
    graceSecs.setDefault(0.25);
    velocityThresholdRadPerSec.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.runPercent(-speedPercent.get());
    graceTimer.reset();
    graceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetClimber", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runPercent(0.0);
    climber.resetPosition();
    graceTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return graceTimer.hasElapsed(graceSecs.get())
        && -climber.getVelocity() < velocityThresholdRadPerSec.get();
  }
}
