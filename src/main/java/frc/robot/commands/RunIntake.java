// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.TunableNumber;

public class RunIntake extends CommandBase {

  private static final TunableNumber forwardsSpeed =
      new TunableNumber("Intake/ForwardsSpeed");
  private static final TunableNumber backwardsSpeed =
      new TunableNumber("Intake/BackwardsSpeed");

  private final Intake intake;
  private final boolean forwards;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, boolean forwards) {
    addRequirements(intake);
    this.intake = intake;
    this.forwards = forwards;

    forwardsSpeed.setDefault(1.0);
    backwardsSpeed.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (forwards) {
      intake.runPercent(forwardsSpeed.get());
    } else {
      intake.runPercent(backwardsSpeed.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
