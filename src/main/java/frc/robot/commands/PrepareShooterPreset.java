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

  private static final TunableNumber lowerFenderBigRpm =
      new TunableNumber("PrepareShooter/LowerFenderBigRPM");
  private static final TunableNumber lowerFenderLitteRpm =
      new TunableNumber("PrepareShooter/LowerFenderLittleRPM");

  private static final TunableNumber upperFenderBigRpm =
      new TunableNumber("PrepareShooter/UpperFenderBigRPM");
  private static final TunableNumber upperFenderLittleRpm =
      new TunableNumber("PrepareShooter/UpperFenderLittleRPM");

  private final Flywheels flywheels;
  private final Hood hood;
  private final ShooterPreset preset;

  /**
   * Creates a new PrepareShooterPreset. Runs the flywheel and sets the hood position for the given
   * preset.
   */
  public PrepareShooterPreset(Flywheels flywheels, Hood hood,
      ShooterPreset preset) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.preset = preset;

    lowerFenderBigRpm.setDefault(500.0);
    lowerFenderLitteRpm.setDefault(2000.0);

    upperFenderBigRpm.setDefault(1700.0);
    upperFenderLittleRpm.setDefault(6000.0);
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
        littleSpeed = lowerFenderLitteRpm.get();
        break;
      case UPPER_FENDER:
        raised = false;
        bigSpeed = upperFenderBigRpm.get();
        littleSpeed = upperFenderLittleRpm.get();
        break;
      default:
        break;
    }
    hood.setRaised(raised);
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
    LOWER_FENDER, UPPER_FENDER
  }
}
