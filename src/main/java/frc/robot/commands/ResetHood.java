// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.TunableNumber;

public class ResetHood extends CommandBase {
  private static final TunableNumber speed =
      new TunableNumber("ResetHood/Speed");
  private static final TunableNumber graceSeconds =
      new TunableNumber("ResetHood/GraceSeconds");
  private static final TunableNumber velocityThreshold =
      new TunableNumber("ResetHood/VelocityThreshold");

  private final Hood hood;
  private final Timer graceTimer = new Timer();

  /** Creates a new ResetHood. */
  public ResetHood(Hood hood) {
    addRequirements(hood);
    this.hood = hood;

    speed.setDefault(0.25);
    graceSeconds.setDefault(0.2);
    velocityThreshold.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.runPercent(-speed.get());
    graceTimer.reset();
    graceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetHood", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.runPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return graceTimer.hasElapsed(graceSeconds.get())
        && -hood.getVelocityDegreesPerSec() < velocityThreshold.get();
  }
}
