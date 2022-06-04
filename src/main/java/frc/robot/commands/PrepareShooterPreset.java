// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.util.TunableNumber;

public class PrepareShooterPreset extends CommandBase {
  public static final TunableNumber lowerFenderRpm =
      new TunableNumber("PrepareShooterPreset/LowerFender/RPM");
  public static final TunableNumber lowerFenderAngle =
      new TunableNumber("PrepareShooterPreset/LowerFender/Angle");
  public static final TunableNumber lowerFenderTower =
      new TunableNumber("PrepareShooterPreset/LowerFender/TowerPercent");

  public static final TunableNumber upperFenderRpm =
      new TunableNumber("PrepareShooterPreset/UpperFender/RPM");
  public static final TunableNumber upperFenderAngle =
      new TunableNumber("PrepareShooterPreset/UpperFender/Angle");
  public static final TunableNumber upperFenderTower =
      new TunableNumber("PrepareShooterPreset/UpperFender/TowerPercent");

  public static final TunableNumber upperTarmacRpm =
      new TunableNumber("PrepareShooterPreset/UpperTarmac/RPM");
  public static final TunableNumber upperTarmacAngle =
      new TunableNumber("PrepareShooterPreset/UpperTarmac/Angle");
  public static final TunableNumber upperTarmacTower =
      new TunableNumber("PrepareShooterPreset/UpperTarmac/TowerPercent");

  public static final TunableNumber upperLaunchpadRpm =
      new TunableNumber("PrepareShooterPreset/UpperLaunchpad/RPM");
  public static final TunableNumber upperLaunchpadAngle =
      new TunableNumber("PrepareShooterPreset/UpperLaunchpad/Angle");
  public static final TunableNumber upperLaunchpadTower =
      new TunableNumber("PrepareShooterPreset/UpperLaunchpad/TowerPercent");

  public static final TunableNumber hangarEjectRpm =
      new TunableNumber("PrepareShooterPreset/HangarEject/RPM");
  public static final TunableNumber hangarEjectAngle =
      new TunableNumber("PrepareShooterPreset/HangarEject/Angle");
  public static final TunableNumber hangarEjectTower =
      new TunableNumber("PrepareShooterPreset/HangarEject/TowerPercent");

  public static final TunableNumber opponentEjectRpm =
      new TunableNumber("PrepareShooterPreset/OpponentEject/RPM");
  public static final TunableNumber opponentEjectAngle =
      new TunableNumber("PrepareShooterPreset/OpponentEject/Angle");
  public static final TunableNumber opponentEjectTower =
      new TunableNumber("PrepareShooterPreset/OpponentEject/TowerPercent");

  public static final TunableNumber mysteryTower =
      new TunableNumber("PrepareShooterPreset/Mystery/TowerPercent");
  public static final TunableNumber mysteryStandsNearRpm =
      new TunableNumber("PrepareShooterPreset/Mystery/StandsNearRPM");
  public static final TunableNumber mysteryStandsNearAngle =
      new TunableNumber("PrepareShooterPreset/Mystery/StandsNearAngle");
  public static final TunableNumber mysteryStandsFarRpm =
      new TunableNumber("PrepareShooterPreset/Mystery/StandsFarRPM");
  public static final TunableNumber mysteryStandsFarAngle =
      new TunableNumber("PrepareShooterPreset/Mystery/StandsFarAngle");
  public static final TunableNumber mysteryAllianceStationRpm =
      new TunableNumber("PrepareShooterPreset/Mystery/AllianceStationRPM");
  public static final TunableNumber mysteryAllianceStationAngle =
      new TunableNumber("PrepareShooterPreset/Mystery/AllianceStationAngle");
  public static final TunableNumber mysteryHumanPlayerRpm =
      new TunableNumber("PrepareShooterPreset/Mystery/HumanPlayerRPM");
  public static final TunableNumber mysteryHumanPlayerAngle =
      new TunableNumber("PrepareShooterPreset/Mystery/HumanPlayerAngle");


  private final Flywheels flywheels;
  private final Hood hood;
  private final Feeder feeder;
  private final ShooterPreset preset;

  /**
   * Creates a new PrepareShooterPreset. Runs the flywheel and sets the hood position for the given
   * preset. Set the feeder to null to disable controlling the tower speed.
   */
  public PrepareShooterPreset(Flywheels flywheels, Hood hood, Feeder feeder,
      ShooterPreset preset) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.feeder = feeder;
    this.preset = preset;

    lowerFenderRpm.setDefault(500.0);
    lowerFenderAngle.setDefault(31.0); // Max angle
    lowerFenderTower.setDefault(0.6);

    upperFenderRpm.setDefault(1220.0);
    upperFenderAngle.setDefault(3.0); // Min angle
    upperFenderTower.setDefault(0.35);

    upperTarmacRpm.setDefault(1190.0);
    upperTarmacAngle.setDefault(24.0);
    upperTarmacTower.setDefault(0.6);

    upperLaunchpadRpm.setDefault(1430.0);
    upperLaunchpadAngle.setDefault(31.0); // Max angle
    upperLaunchpadTower.setDefault(0.6);

    hangarEjectRpm.setDefault(200.0);
    hangarEjectAngle.setDefault(31.0); // Max angle
    hangarEjectTower.setDefault(0.6);

    opponentEjectRpm.setDefault(800.0);
    opponentEjectAngle.setDefault(3.0); // Min angle
    opponentEjectTower.setDefault(0.6);

    mysteryTower.setDefault(0.6);
    mysteryStandsNearRpm.setDefault(1800.0);
    mysteryStandsNearAngle.setDefault(31.0);
    mysteryStandsFarRpm.setDefault(2650.0);
    mysteryStandsFarAngle.setDefault(40.0);
    mysteryAllianceStationRpm.setDefault(1500.0);
    mysteryAllianceStationAngle.setDefault(31.0);
    mysteryHumanPlayerRpm.setDefault(2100.0);
    mysteryHumanPlayerAngle.setDefault(31.0);
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
      case UPPER_LAUNCHPAD:
        flywheelSpeed = upperLaunchpadRpm.get();
        hoodAngle = upperLaunchpadAngle.get();
        towerSpeed = upperLaunchpadTower.get();
        break;
      case HANGAR_EJECT:
        flywheelSpeed = hangarEjectRpm.get();
        hoodAngle = hangarEjectAngle.get();
        towerSpeed = hangarEjectTower.get();
        break;
      case OPPONENT_EJECT:
        flywheelSpeed = opponentEjectRpm.get();
        hoodAngle = opponentEjectAngle.get();
        towerSpeed = opponentEjectTower.get();
        break;
      case MYSTERY_STANDS_NEAR:
        flywheelSpeed = mysteryStandsNearRpm.get();
        hoodAngle = mysteryStandsNearAngle.get();
        towerSpeed = mysteryTower.get();
        break;
      case MYSTERY_STANDS_FAR:
        flywheelSpeed = mysteryStandsFarRpm.get();
        hoodAngle = mysteryStandsFarAngle.get();
        towerSpeed = mysteryTower.get();
        break;
      case MYSTERY_ALLIANCE_STATION:
        flywheelSpeed = mysteryAllianceStationRpm.get();
        hoodAngle = mysteryAllianceStationAngle.get();
        towerSpeed = mysteryTower.get();
        break;
      case MYSTERY_HUMAN_PLAYER:
        flywheelSpeed = mysteryHumanPlayerRpm.get();
        hoodAngle = mysteryHumanPlayerAngle.get();
        towerSpeed = mysteryTower.get();
        break;
      default:
        break;
    }
    flywheels.runVelocity(flywheelSpeed);
    hood.moveToAngle(hoodAngle);
    if (feeder != null) {
      feeder.requestTowerShootPercent(towerSpeed);
    }
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
    LOWER_FENDER, UPPER_FENDER, UPPER_TARMAC, UPPER_LAUNCHPAD, HANGAR_EJECT, OPPONENT_EJECT, MYSTERY_STANDS_NEAR, MYSTERY_STANDS_FAR, MYSTERY_ALLIANCE_STATION, MYSTERY_HUMAN_PLAYER
  }
}
