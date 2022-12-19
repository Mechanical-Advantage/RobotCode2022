// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveWithJoysticks extends CommandBase {
  private static final TunableNumber deadband = new TunableNumber("DriveWithJoysticks/Deadband");
  private static final TunableNumber sniperLevel =
      new TunableNumber("DriveWithJoysticks/SniperLevel");
  private static final TunableNumber maxAcceleration =
      new TunableNumber("DriveWithJoysticks/MaxAcceleration"); // Percent velocity per second
  private static final TunableNumber maxJerk =
      new TunableNumber("DriveWithJoysticks/MaxJerk"); // Percent velocity per second^2
  private static final TunableNumber curvatureThreshold =
      new TunableNumber("DriveWithJoysticks/CurvatureThreshold"); // Where to transition to full
  // curvature
  private static final TunableNumber curvatureArcadeTurnScale =
      new TunableNumber("DriveWithJoysticks/CurvatureArcadeTurnScale"); // Arcade turning scale
  // factor
  private final Drive drive;
  private final Supplier<String> modeSupplier;
  private final Supplier<String> demoSpeedLimitSupplier;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Double> rightYSupplier;
  private final Supplier<Boolean> sniperModeSupplier;

  private final AxisProcessor leftXProcessor = new AxisProcessor();
  private final AxisProcessor leftYProcessor = new AxisProcessor();
  private final AxisProcessor rightXProcessor = new AxisProcessor();
  private final AxisProcessor rightYProcessor = new AxisProcessor();

  /** Creates a new DriveWithJoysticks. Drives based on the joystick values. */
  public DriveWithJoysticks(
      Drive drive,
      Supplier<String> modeSupplier,
      Supplier<String> demoSpeedLimitSupplier,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Double> rightYSupplier,
      Supplier<Boolean> sniperModeSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.modeSupplier = modeSupplier;
    this.demoSpeedLimitSupplier = demoSpeedLimitSupplier;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;
    this.sniperModeSupplier = sniperModeSupplier;

    deadband.setDefault(0.08);
    sniperLevel.setDefault(0.5);
    maxAcceleration.setDefault(99999.0);
    maxJerk.setDefault(200.0);
    curvatureThreshold.setDefault(0.15);
    curvatureArcadeTurnScale.setDefault(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftXProcessor.reset(leftXSupplier.get());
    leftYProcessor.reset(leftYSupplier.get());
    rightXProcessor.reset(rightXSupplier.get());
    rightYProcessor.reset(rightYSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  @SuppressWarnings("unused")
  public void execute() {
    double leftXValue = leftXProcessor.process(leftXSupplier.get());
    double leftYValue = leftYProcessor.process(leftYSupplier.get());
    double rightXValue = rightXProcessor.process(rightXSupplier.get());
    double rightYValue = rightYProcessor.process(rightYSupplier.get());

    WheelSpeeds speeds = new WheelSpeeds(0.0, 0.0);
    switch (modeSupplier.get()) {
      case "Tank":
        speeds = new WheelSpeeds(leftYValue, rightYValue);
        break;

      case "Split Arcade":
        speeds = WheelSpeeds.fromArcade(leftYValue, rightXValue);
        break;

      case "Curvature":
        WheelSpeeds arcadeSpeeds =
            WheelSpeeds.fromArcade(leftYValue, rightXValue * curvatureArcadeTurnScale.get());
        WheelSpeeds curvatureSpeeds = WheelSpeeds.fromCurvature(leftYValue, rightXValue);
        double hybridScale = Math.abs(leftYValue) / curvatureThreshold.get();
        hybridScale = hybridScale > 1 ? 1 : hybridScale;
        speeds =
            new WheelSpeeds(
                curvatureSpeeds.left * hybridScale + arcadeSpeeds.left * (1 - hybridScale),
                curvatureSpeeds.right * hybridScale + arcadeSpeeds.right * (1 - hybridScale));
        break;
    }

    if (sniperModeSupplier.get()) {
      speeds = new WheelSpeeds(speeds.left * sniperLevel.get(), speeds.right * sniperLevel.get());
    }

    double demoSpeedLimit = 1.0;
    switch (demoSpeedLimitSupplier.get()) {
      case "Fast Speed (70%)":
        demoSpeedLimit = 0.7;
        break;
      case "Medium Speed (30%)":
        demoSpeedLimit = 0.3;
        break;
      case "Slow Speed (15%)":
        demoSpeedLimit = 0.15;
        break;
      default:
        break;
    }

    double leftPercent = MathUtil.clamp(speeds.left, -1.0, 1.0) * demoSpeedLimit;
    double rightPercent = MathUtil.clamp(speeds.right, -1.0, 1.0) * demoSpeedLimit;

    Logger.getInstance().recordOutput("ActiveCommands/DriveWithJoysticks", true);
    Logger.getInstance().recordOutput("DriveWithJoysticks/LeftPercent", leftPercent);
    Logger.getInstance().recordOutput("DriveWithJoysticks/RightPercent", rightPercent);
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

  public static double getDeadband() {
    return deadband.get();
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

    public static WheelSpeeds fromCurvature(double baseSpeed, double turnSpeed) {
      turnSpeed = Math.abs(baseSpeed) * turnSpeed;
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }
  }

  /** Cleans up a series of axis value (deadband + squaring + profile) */
  public static class AxisProcessor {
    private TrapezoidProfile.State state = new TrapezoidProfile.State();

    public void reset(double value) {
      state = new TrapezoidProfile.State(value, 0.0);
    }

    public double process(double value) {
      double scaledValue = 0.0;
      if (Math.abs(value) > deadband.get()) {
        scaledValue = (Math.abs(value) - deadband.get()) / (1 - deadband.get());
        scaledValue = Math.copySign(scaledValue * scaledValue, value);
      }
      TrapezoidProfile profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxAcceleration.get(), maxJerk.get()),
              new TrapezoidProfile.State(scaledValue, 0.0),
              state);
      state = profile.calculate(Constants.loopPeriodSecs);
      return state.position;
    }
  }
}
