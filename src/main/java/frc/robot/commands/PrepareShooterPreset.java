// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class PrepareShooterPreset extends CommandBase {
  public static final TunableNumber lowerFenderRpm =
      new TunableNumber("PrepareShooterPreset/LowerFender/RPM");
  public static final TunableNumber upperFenderRpm =
      new TunableNumber("PrepareShooterPreset/UpperFender/RPM");
  public static final TunableNumber upperTarmacRpm =
      new TunableNumber("PrepareShooterPreset/UpperTarmac/RPM");

  public static final TunableNumber lowerFenderAngle =
      new TunableNumber("PrepareShooterPreset/LowerFender/Angle");
  public static final TunableNumber upperFenderAngle =
      new TunableNumber("PrepareShooterPreset/UpperFender/Angle");
  public static final TunableNumber upperTarmacAngle =
      new TunableNumber("PrepareShooterPreset/UpperTarmac/Angle");

  public static final TunableNumber lowerFenderTower =
      new TunableNumber("PrepareShooterPreset/LowerFender/TowerPercent");
  public static final TunableNumber upperFenderTower =
      new TunableNumber("PrepareShooterPreset/UpperFender/TowerPercent");
  public static final TunableNumber upperTarmacTower =
      new TunableNumber("PrepareShooterPreset/UpperTarmac/TowerPercent");

  private final Flywheels flywheels;
  private final Hood hood;
  private final Tower tower;
  private final ShooterPreset preset;

  /**
   * Creates a new PrepareShooterPreset. Runs the flywheel and sets the hood position for the given
   * preset.
   */
  public PrepareShooterPreset(Flywheels flywheels, Hood hood, Tower tower,
      ShooterPreset preset) {
    addRequirements(flywheels);
    this.flywheels = flywheels;
    this.hood = hood;
    this.tower = tower;
    this.preset = preset;

    lowerFenderRpm.setDefault(500.0);
    upperFenderRpm.setDefault(1140.0);
    upperTarmacRpm.setDefault(1250.0);

    lowerFenderAngle.setDefault(31.0); // Max angle
    upperFenderAngle.setDefault(6.0); // Min angle
    upperTarmacAngle.setDefault(25.0);

    lowerFenderTower.setDefault(1.0);
    upperFenderTower.setDefault(0.35);
    upperTarmacTower.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double flywheelSpeed = 0.0, hoodAngle = 0.0, towerSpeed = 0.0;
    switch (preset) {
      case LOWER_FENDER:
        flywheelSpeed = lowerFenderRpm.get();
        hoodAngle = lowerFenderAngle.get();
        towerSpeed = lowerFenderTower.get();
        break;
      case UPPER_FENDER:
        flywheelSpeed = upperFenderRpm.get();
        hoodAngle = upperFenderAngle.get();
        towerSpeed = upperFenderTower.get();
        break;
      case UPPER_TARMAC:
        flywheelSpeed = upperTarmacRpm.get();
        hoodAngle = upperTarmacAngle.get();
        towerSpeed = upperTarmacTower.get();
        break;
      default:
        break;
    }
    flywheels.runVelocity(flywheelSpeed);
    hood.moveToAngle(hoodAngle);
    tower.requestShootPercent(towerSpeed);
    Logger.getInstance().recordOutput("ActiveCommands/PrepareShooterPreset",
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheels.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum ShooterPreset {
    LOWER_FENDER, UPPER_FENDER, UPPER_TARMAC
  }
}
