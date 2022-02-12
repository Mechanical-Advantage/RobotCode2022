// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class RunClimber extends CommandBase {

  private final Climber climber;
  private final Supplier<Double> axisSupplier;

  /** Creates a new RunClimber. */
  public RunClimber(Climber climber, Supplier<Double> axisSupplier) {
    addRequirements(climber);
    this.climber = climber;
    this.axisSupplier = axisSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = axisSupplier.get();
    double scaledValue = 0.0;
    if (Math.abs(value) > DriveWithJoysticks.getDeadband()) {
      scaledValue = (Math.abs(value) - DriveWithJoysticks.getDeadband())
          / (1 - DriveWithJoysticks.getDeadband());
      scaledValue = Math.copySign(scaledValue * scaledValue, value);
    }
    climber.runPercent(scaledValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
