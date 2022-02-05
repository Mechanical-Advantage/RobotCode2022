// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class Shoot extends CommandBase {

  private static final TunableNumber towerSpeed =
      new TunableNumber("Shooter/TowerSpeed");
  private static final TunableNumber kickerSpeed =
      new TunableNumber("Shooter/KickerSpeed");

  private Tower tower;
  private Kicker kicker;

  /** Creates a new Shoot. Runs the tower and kicker to fire cargo. */
  public Shoot(Tower tower, Kicker kicker) {
    addRequirements(tower, kicker);
    this.tower = tower;
    this.kicker = kicker;

    towerSpeed.setDefault(1.0);
    kickerSpeed.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.runPercent(towerSpeed.get());
    kicker.runPercent(kickerSpeed.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("Shoot/Active", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stop();
    kicker.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
