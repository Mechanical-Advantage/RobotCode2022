// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.oi.OverrideOI.VisionLedMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.util.TunableNumber;

public class IdleHood extends CommandBase {
  private static final TunableNumber nearThresholdMeters =
      new TunableNumber("IdleHood/NearThreshold");
  private static final TunableNumber farThresholdMeters =
      new TunableNumber("IdleHood/FarThreshold");

  private final Hood hood;
  private final Drive drive;
  private final Supplier<VisionLedMode> visionModeSupplier;

  /** Creates a new IdleHood. */
  public IdleHood(Hood hood, Drive drive,
      Supplier<VisionLedMode> visionModeSupplier) {
    addRequirements(hood);
    this.hood = hood;
    this.drive = drive;
    this.visionModeSupplier = visionModeSupplier;

    nearThresholdMeters.setDefault(3.0);
    nearThresholdMeters.setDefault(3.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionModeSupplier.get() == VisionLedMode.ALWAYS_OFF) {
      return;
    }

    HoodState hoodState = hood.getState();
    if (hoodState == HoodState.MOVING) {
      return;
    }

    double distance =
        drive.getPose().getTranslation().getDistance(FieldConstants.hubCenter);
    boolean shouldRaise;
    if (hoodState == HoodState.RAISED) {
      shouldRaise = distance > nearThresholdMeters.get();
    } else {
      shouldRaise = distance > farThresholdMeters.get();
    }
    hood.setRaised(shouldRaise);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
