// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class RunTower extends CommandBase {

  private static final TunableNumber upSpeed =
      new TunableNumber("RunTower/UpSpeed");
  private static final TunableNumber downSpeed =
      new TunableNumber("RunTower/DownSpeed");

  private final Tower tower;
  private final boolean up;

  /**
   * Creates a new RunTower. Runs the tower up or down, intended for operator controls.
   */
  public RunTower(Tower tower, boolean up) {
    addRequirements(tower);
    this.tower = tower;
    this.up = up;

    upSpeed.setDefault(1.0);
    downSpeed.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (up) {
      tower.runPercent(upSpeed.get());
    } else {
      tower.runPercent(downSpeed.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
