// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class HopperEject extends CommandBase {
  private static final TunableNumber kickerSpeed =
      new TunableNumber("HopperEject/KickerSpeed");
  private static final TunableNumber towerSpeed =
      new TunableNumber("HopperEject/TowerSpeed");
  private static final TunableNumber hopperSpeed =
      new TunableNumber("HopperEject/HopperSpeed");

  private final Intake intake;
  private final Tower tower;
  private final Kicker kicker;

  /**
   * Creates a new HopperEject. Runs the feed system backwards to eject the cargo with the intake
   * retracted.
   */
  public HopperEject(Intake intake, Tower tower, Kicker kicker) {
    addRequirements(intake, tower, kicker);
    this.intake = intake;
    this.tower = tower;
    this.kicker = kicker;

    kickerSpeed.setDefault(-1.0);
    towerSpeed.setDefault(-1.0);
    hopperSpeed.setDefault(-1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.retract();
    kicker.runPercent(kickerSpeed.get());
    tower.runPercent(towerSpeed.get());
    intake.runHopperPercent(hopperSpeed.get());
    intake.runRollerPercent(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stop();
    tower.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
