// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class DriveWithJoysticks extends CommandBase {
  private static final double deadband = 0.08;
  private static final double curvatureThreshold = 0.15; // Where to transition to full curvature
  private static final double curvatureArcadeTurnScale = 0.5; // Arcade turning scale factor
  private final Drive driveTrain;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Double> rightYSupplier;

  private final SendableChooser<JoystickMode> modeChooser =
      new SendableChooser<JoystickMode>();

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier,

      Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier,

      Supplier<Double> rightYSupplier) {
    addRequirements(drive);
    this.driveTrain = drive;
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

  /** Apply deadband and square. */
  private double processAxis(double value) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    double scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
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
    switch (modeChooser.getSelected()) {
      case TANK:
        speeds = new WheelSpeeds(leftYValue, rightYValue);
        break;

      case SPLIT_ARCADE:
        speeds = WheelSpeeds.fromArcade(leftYValue, rightXValue);
        break;

      case CURVATURE:
        WheelSpeeds arcadeSpeeds = WheelSpeeds.fromArcade(leftYValue,
            rightXValue * curvatureArcadeTurnScale);
        WheelSpeeds curvatureSpeeds =
            WheelSpeeds.fromCurvature(leftYValue, rightXValue);
        double hybridScale = Math.abs(leftYValue) / curvatureThreshold;
        hybridScale = hybridScale > 1 ? 1 : hybridScale;
        speeds = new WheelSpeeds(
            curvatureSpeeds.left * hybridScale
                + arcadeSpeeds.left * (1 - hybridScale),
            curvatureSpeeds.right * hybridScale
                + arcadeSpeeds.right * (1 - hybridScale));
        break;
    }

    double leftPercent = MathUtil.clamp(speeds.left, -1.0, 1.0);
    double rightPercent = MathUtil.clamp(speeds.right, -1.0, 1.0);
    Logger.getInstance().recordOutput("DriveWithJoysticks/LeftPercent",
        leftPercent);
    Logger.getInstance().recordOutput("DriveWithJoysticks/RightPercent",
        rightPercent);
    driveTrain.drivePercent(leftPercent, rightPercent);
  }

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

  private static enum JoystickMode {
    CURVATURE, SPLIT_ARCADE, TANK
  }
}
