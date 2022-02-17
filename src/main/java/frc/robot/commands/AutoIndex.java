// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class AutoIndex extends CommandBase {
  private static final TunableNumber graceSecs =
      new TunableNumber("AutoIndex/GraceSecs");
  private static final TunableNumber speed =
      new TunableNumber("AutoIndex/Speed");

  private final Tower tower;
  private final Timer graceTimer = new Timer();
  private final Supplier<Boolean> disableSupplier;

  /** Creates a new AutoIndex. Runs the tower forwards whenever the cargo sensor is tripped. */
  public AutoIndex(Tower tower) {
    this(tower, () -> false);
  }

  /** Creates a new AutoIndex. Runs the tower forwards whenever the cargo sensor is tripped. */
  public AutoIndex(Tower tower, Supplier<Boolean> disableSupplier) {
    addRequirements(tower);
    this.tower = tower;
    this.disableSupplier = disableSupplier;

    graceSecs.setDefault(0);
    graceTimer.reset();
    graceTimer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (disableSupplier.get()) {
      tower.stop();
      return;
    }

    Logger.getInstance().recordOutput("ActiveCommands/AutoIndex", true);

    if (tower.getLowerCargoSensor()) {
      graceTimer.reset();
    }
    if (graceTimer.hasElapsed(graceSecs.get())) {
      tower.runPercent(speed.get());
    } else {
      tower.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
