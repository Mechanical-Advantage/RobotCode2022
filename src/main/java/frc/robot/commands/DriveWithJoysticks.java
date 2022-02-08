// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableNumber;

public class DriveWithJoysticks extends CommandBase {
  private static final TunableNumber deadband =
      new TunableNumber("DriveWithJoysticks/Deadband");
  private static final TunableNumber sniperLevel =
      new TunableNumber("DriveWithJoysticks/SniperLevel");
  private static final TunableNumber curvatureThreshold =
      new TunableNumber("DriveWithJoysticks/CurvatureThreshold"); // Where to transition to full
                                                                  // curvature
  private static final TunableNumber curvatureArcadeTurnScale =
      new TunableNumber("DriveWithJoysticks/CurvatureArcadeTurnScale"); // Arcade turning scale
                                                                        // factor
  private final Drive drive;
  private final Supplier<String> modeSupplier;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Double> rightYSupplier;
  private final Supplier<Boolean> sniperModeSupplier;

  /** Creates a new DriveWithJoysticks. Drives based on the joystick values. */
  public DriveWithJoysticks(Drive drive, Supplier<String> modeSupplier,
      Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier, Supplier<Double> rightYSupplier,
      Supplier<Boolean> sniperModeSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.modeSupplier = modeSupplier;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;
    this.sniperModeSupplier = sniperModeSupplier;

    deadband.setDefault(0.08);
    sniperLevel.setDefault(0.5);
    curvatureThreshold.setDefault(0.15);
    curvatureArcadeTurnScale.setDefault(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /** Apply deadband and square. */
  private double processAxis(double value) {
    if (Math.abs(value) < deadband.get()) {
      return 0.0;
    }
    double scaledValue =
        (Math.abs(value) - deadband.get()) / (1 - deadband.get());
    return Math.copySign(scaledValue * scaledValue, value);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  @SuppressWarnings("unused")
  public void execute() {
    double leftXValue = processAxis(leftXSupplier.get());
    double leftYValue = processAxis(leftYSupplier.get());
    double rightXValue = processAxis(rightXSupplier.get());
    double rightYValue = processAxis(rightYSupplier.get());

    WheelSpeeds speeds = new WheelSpeeds(0.0, 0.0);
    switch (modeSupplier.get()) {
      case "Tank":
        speeds = new WheelSpeeds(leftYValue, rightYValue);
        break;

      case "Split Arcade":
        speeds = WheelSpeeds.fromArcade(leftYValue, rightXValue);
        break;

      case "Curvature":
        WheelSpeeds arcadeSpeeds = WheelSpeeds.fromArcade(leftYValue,
            rightXValue * curvatureArcadeTurnScale.get());
        WheelSpeeds curvatureSpeeds =
            WheelSpeeds.fromCurvature(leftYValue, rightXValue);
        double hybridScale = Math.abs(leftYValue) / curvatureThreshold.get();
        hybridScale = hybridScale > 1 ? 1 : hybridScale;
        speeds = new WheelSpeeds(
            curvatureSpeeds.left * hybridScale
                + arcadeSpeeds.left * (1 - hybridScale),
            curvatureSpeeds.right * hybridScale
                + arcadeSpeeds.right * (1 - hybridScale));
        break;
    }

    if (sniperModeSupplier.get()) {
      speeds = new WheelSpeeds(speeds.left * sniperLevel.get(),
          speeds.right * sniperLevel.get());
    }

    double leftPercent = MathUtil.clamp(speeds.left, -1.0, 1.0);
    double rightPercent = MathUtil.clamp(speeds.right, -1.0, 1.0);
    Logger.getInstance().recordOutput("DriveWithJoysticks/LeftPercent",
        leftPercent);
    Logger.getInstance().recordOutput("DriveWithJoysticks/RightPercent",
        rightPercent);
    drive.drivePercent(leftPercent, rightPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Represents a left and right percentage. */
  private static class WheelSpeeds {
    public double left;
    public double right;

    public WheelSpeeds(double left, double right) {
      this.left = left;
      this.right = right;
    }

    public static WheelSpeeds fromArcade(double baseSpeed, double turnSpeed) {
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }

    public static WheelSpeeds fromCurvature(double baseSpeed,
        double turnSpeed) {
      turnSpeed = Math.abs(baseSpeed) * turnSpeed;
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }
  }
}
