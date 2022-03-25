// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.TunableNumber;

public class ResetClimber extends CommandBase {
  private static final TunableNumber initialRaiseSecs =
      new TunableNumber("ResetClimber/InitialRaise/Secs");
  private static final TunableNumber initialRaiseVolts =
      new TunableNumber("ResetClimber/InitialRaise/Volts");

  private static final TunableNumber pullDownGraceSecs =
      new TunableNumber("ResetClimber/PullDown/GraceSecs");
  private static final TunableNumber pullDownVelocityThreshold =
      new TunableNumber("ResetClimber/PullDown/VelocityThreshold");
  private static final TunableNumber pullDownVolts =
      new TunableNumber("ResetClimber/PullDown/Volts");

  private static final TunableNumber releaseTensionSecs =
      new TunableNumber("ResetClimber/ReleaseTension/Secs");
  private static final TunableNumber releaseTensionVolts =
      new TunableNumber("ResetClimber/ReleaseTension/Volts");

  private final Climber climber;
  private final Timer timer = new Timer();
  private ResetStage stage = ResetStage.INACTIVE;

  /** Creates a new ResetClimber. */
  public ResetClimber(Climber climber) {
    addRequirements(climber);
    this.climber = climber;

    initialRaiseSecs.setDefault(0.4);
    initialRaiseVolts.setDefault(0.6);

    pullDownGraceSecs.setDefault(0.8);
    pullDownVelocityThreshold.setDefault(0.03);
    pullDownVolts.setDefault(0.6);

    releaseTensionSecs.setDefault(0.1);
    releaseTensionVolts.setDefault(0.6);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    stage = ResetStage.INITIAL_RAISE;
    climber.runVoltage(initialRaiseVolts.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetClimber", true);

    switch (stage) {
      case INITIAL_RAISE:
        if (timer.hasElapsed(initialRaiseSecs.get())) {
          timer.reset();
          stage = ResetStage.PULL_DOWN;
          climber.runVoltage(-pullDownVolts.get());
        }
        break;

      case PULL_DOWN:
        if (timer.hasElapsed(pullDownGraceSecs.get()) && Math
            .abs(climber.getVelocity()) < pullDownVelocityThreshold.get()) {
          timer.reset();
          stage = ResetStage.RELEASE_TENSION;
          climber.runVoltage(releaseTensionVolts.get());
          climber.resetPosition();
        }
        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runVoltage(0.0);
    timer.stop();
    stage = ResetStage.INACTIVE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == ResetStage.RELEASE_TENSION
        && timer.hasElapsed(releaseTensionSecs.get());
  }

  private static enum ResetStage {
    INACTIVE, INITIAL_RAISE, PULL_DOWN, RELEASE_TENSION
  }
}
