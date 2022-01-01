// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
  private final DriveTrain driveTrain;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Double> rightYSupplier;

  private final SendableChooser<JoystickMode> modeChooser =
      new SendableChooser<JoystickMode>();

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(DriveTrain driveTrain,
      Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier, Supplier<Double> rightYSupplier) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;

    modeChooser.setDefaultOption("Curvature", JoystickMode.CURVATURE);
    modeChooser.addOption("Split Arcade", JoystickMode.SPLIT_ARCADE);
    modeChooser.addOption("Tank", JoystickMode.TANK);
    SmartDashboard.putData("Joystick Mode", modeChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static enum JoystickMode {
    CURVATURE, SPLIT_ARCADE, TANK
  }
}
