// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.TunableNumber;

public class PrepareShooter extends CommandBase {

  private static final Map<ShooterPreset, Boolean> raisedMap =
      Map.of(ShooterPreset.LOWER_FENDER, false, ShooterPreset.UPPER_FENDER,
          true, ShooterPreset.UPPER_TARMAC, true);
  private static final Map<ShooterPreset, TunableNumber> bigRpmMap =
      Map.of(ShooterPreset.LOWER_FENDER,
          new TunableNumber("PrepareShooter/LowerFenderBigRPM"),
          ShooterPreset.UPPER_FENDER,
          new TunableNumber("PrepareShooter/UpperFenderBigRPM"),
          ShooterPreset.UPPER_TARMAC,
          new TunableNumber("PrepareShooter/UpperTarmacBigRPM"));
  private static final Map<ShooterPreset, TunableNumber> littleRpmMap =
      Map.of(ShooterPreset.LOWER_FENDER,
          new TunableNumber("PrepareShooter/LowerFenderLittleRPM"),
          ShooterPreset.UPPER_FENDER,
          new TunableNumber("PrepareShooter/UpperFenderLittleRPM"),
          ShooterPreset.UPPER_TARMAC,
          new TunableNumber("PrepareShooter/UpperTarmacLittleRPM"));

  private final Flywheels flywheels;
  private final Hood hood;
  private final ShooterPreset preset;

  /**
   * Creates a new PrepareShooter. Runs the flywheel and sets the hood position for the given
   * preset.
   */
  public PrepareShooter(Flywheels flywheels, Hood hood, ShooterPreset preset) {
    addRequirements(flywheels, hood);
    this.flywheels = flywheels;
    this.hood = hood;
    this.preset = preset;

    bigRpmMap.get(ShooterPreset.LOWER_FENDER).setDefault(0);
    littleRpmMap.get(ShooterPreset.LOWER_FENDER).setDefault(0);

    bigRpmMap.get(ShooterPreset.UPPER_FENDER).setDefault(2000);
    littleRpmMap.get(ShooterPreset.UPPER_FENDER).setDefault(2000);

    bigRpmMap.get(ShooterPreset.UPPER_TARMAC).setDefault(0);
    littleRpmMap.get(ShooterPreset.UPPER_TARMAC).setDefault(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setRaised(raisedMap.get(preset));
    flywheels.runVelocity(bigRpmMap.get(preset).get(),
        littleRpmMap.get(preset).get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
