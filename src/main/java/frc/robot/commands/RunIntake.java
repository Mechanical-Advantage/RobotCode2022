// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class RunIntake extends CommandBase {

  private static final TunableNumber rollerForwardsSpeed =
      new TunableNumber("RunIntake/RollerForwardsSpeed");
  private static final TunableNumber hopperForwardsSpeed =
      new TunableNumber("RunIntake/HopperForwardsSpeed");
  private static final TunableNumber towerForwardsSpeed =
      new TunableNumber("RunIntake/TowerForwardsSpeed");
  private static final TunableNumber kickerForwardsSpeed =
      new TunableNumber("RunIntake/KickerForwardsSpeed");

  private static final TunableNumber rollerBackwardsSpeed =
      new TunableNumber("RunIntake/RollerBackwardsSpeed");
  private static final TunableNumber hopperBackwardsSpeed =
      new TunableNumber("RunIntake/HopperBackwardsSpeed");

  private final boolean forwards;
  private final Intake intake;
  private final Tower tower;
  private final Kicker kicker;

  /**
   * Creates a new RunIntake. Runs the intake forwards or backwards, intended for operator controls.
   */
  public RunIntake(boolean forwards, Intake intake, Tower tower,
      Kicker kicker) {
    addRequirements(intake, tower, kicker);
    this.forwards = forwards;
    this.intake = intake;
    this.tower = tower;
    this.kicker = kicker;

    rollerForwardsSpeed.setDefault(1.0);
    hopperForwardsSpeed.setDefault(1.0);
    towerForwardsSpeed.setDefault(1.0);
    kickerForwardsSpeed.setDefault(1.0);

    rollerBackwardsSpeed.setDefault(1.0);
    hopperBackwardsSpeed.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (forwards) {
      intake.runRollerPercent(rollerForwardsSpeed.get());
      intake.runHopperPercent(hopperForwardsSpeed.get());
      tower.runPercent(towerForwardsSpeed.get());
      kicker.runPercent(kickerForwardsSpeed.get());
    } else {
      intake.runRollerPercent(rollerBackwardsSpeed.get());
      intake.runHopperPercent(hopperBackwardsSpeed.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/RunIntake", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    tower.stop();
    kicker.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
