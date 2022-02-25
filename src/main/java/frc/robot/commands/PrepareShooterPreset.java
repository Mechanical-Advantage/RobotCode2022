// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.TunableNumber;

public class PrepareShooterPreset extends CommandBase {
  private static final double littleMultiplier = 1.1 * 3.0;

  private static final TunableNumber lowerFenderBigRpm =
      new TunableNumber("PrepareShooter/LowerFenderBigRPM");
  private static final TunableNumber upperFenderBigRpm =
      new TunableNumber("PrepareShooter/UpperFenderBigRPM");
  private static final TunableNumber upperTarmacBigRpm =
      new TunableNumber("PrepareShooter/UpperTarmacBigRPM");

  private final Flywheels flywheels;
  private final Hood hood;
  private final ShooterPreset preset;

  /**
   * Creates a new PrepareShooterPreset. Runs the flywheel and sets the hood position for the given
   * preset.
   */
  public PrepareShooterPreset(Flywheels flywheels, Hood hood,
      ShooterPreset preset) {
    addRequirements(flywheels);
    this.flywheels = flywheels;
    this.hood = hood;
    this.preset = preset;

    lowerFenderBigRpm.setDefault(1175.0);
    upperFenderBigRpm.setDefault(1100.0);
    upperTarmacBigRpm.setDefault(2650.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean raised = false;
    double bigSpeed = 0.0, littleSpeed = 0.0;
    switch (preset) {
      case LOWER_FENDER:
        raised = true;
        bigSpeed = lowerFenderBigRpm.get();
        break;
      case UPPER_FENDER:
        raised = false;
        bigSpeed = upperFenderBigRpm.get();
        break;
      case UPPER_TARMAC:
        raised = false;
        bigSpeed = upperTarmacBigRpm.get();
        break;
      default:
        break;
    }
    hood.requestShootPosition(raised);
    littleSpeed = bigSpeed * littleMultiplier;
    flywheels.runVelocity(bigSpeed, littleSpeed);
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
