// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.commands.DriveWithJoysticks.AxisProcessor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TunableNumber;

public class AutoAim extends CommandBase {
  private final Drive drive;
  private final RobotState robotState;
  private final Vision vision;
  private final Translation2d target;
  private final Supplier<Double> speedSupplier;

  private final PIDController controller;
  private final Timer toleranceTimer = new Timer();
  private final AxisProcessor axisProcessor = new AxisProcessor();

  private static final TunableNumber kP = new TunableNumber("AutoAim/kP");
  private static final TunableNumber kI = new TunableNumber("AutoAim/kI");
  private static final TunableNumber kD = new TunableNumber("AutoAim/kD");
  private static final TunableNumber integralMaxError =
      new TunableNumber("AutoAim/IntegralMaxError");
  private static final TunableNumber minVelocity =
      new TunableNumber("AutoAim/MinVelocity");
  private static final TunableNumber toleranceDegrees =
      new TunableNumber("AutoAim/ToleranceDegrees");
  private static final TunableNumber toleranceTime =
      new TunableNumber("AutoAim/ToleranceTime");

  /**
   * Creates a new AutoAim. Points towards the center of the field using odometry data. Set the
   * vision to null to disable controlling LEDs, target to null to point to the hub, or
   * speedSupplier to null to disable operator controls.
   */
  public AutoAim(Drive drive, RobotState robotState, Vision vision,
      Translation2d target, Supplier<Double> speedSupplier) {
    addRequirements(drive);
    if (vision != null) {
      addRequirements(vision);
    }
    this.drive = drive;
    this.robotState = robotState;
    this.vision = vision;
    this.target = target;
    this.speedSupplier = speedSupplier == null ? () -> 0.0 : speedSupplier;

    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
        kP.setDefault(0.008);
        kI.setDefault(0.0);
        kD.setDefault(0.0005);
        integralMaxError.setDefault(0.0);
        minVelocity.setDefault(0.0);
        toleranceDegrees.setDefault(3.0);
        toleranceTime.setDefault(0.3);
        break;
      case ROBOT_2020:
        kP.setDefault(0.018);
        kI.setDefault(0.005);
        kD.setDefault(0.001);
        integralMaxError.setDefault(4);
        minVelocity.setDefault(0.02);
        toleranceDegrees.setDefault(1.5);
        toleranceTime.setDefault(0.1);
        break;
      case ROBOT_SIMBOT:
        kP.setDefault(0.018);
        kI.setDefault(0.0);
        kD.setDefault(0.001);
        integralMaxError.setDefault(0.0);
        minVelocity.setDefault(0.0);
        toleranceDegrees.setDefault(1.5);
        toleranceTime.setDefault(0.25);
        break;
      default:
        break;
    }

    controller = new PIDController(kP.get(), kI.get(), kD.get(),
        Constants.loopPeriodSecs);
    controller.setTolerance(toleranceDegrees.get());
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    toleranceTimer.reset();
    toleranceTimer.start();
    if (vision != null) {
      vision.setForceLeds(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update tunable numbers
    if (kP.hasChanged()) {
      controller.setP(kP.get());
    }
    if (kI.hasChanged()) {
      controller.setI(kI.get());
    }
    if (kD.hasChanged()) {
      controller.setD(kD.get());
    }
    if (toleranceDegrees.hasChanged()) {
      controller.setTolerance(toleranceDegrees.get());
    }

    // Update setpoint
    if (target == null) {
      controller.setSetpoint(
          getTargetRotation(robotState.getLatestPose().getTranslation())
              .getDegrees());
    } else {
      controller.setSetpoint(
          getTargetRotation(robotState.getLatestPose().getTranslation(), target,
              false).getDegrees());
    }

    // Check if in tolerance
    if (!controller.atSetpoint()) {
      toleranceTimer.reset();
    }

    // Update output speeds
    if (Math.abs(controller.getPositionError()) < integralMaxError.get()) {
      controller.setI(kI.get());
    } else {
      controller.setI(0);
    }
    double angularSpeed =
        controller.calculate(robotState.getLatestRotation().getDegrees());
    if (Math.abs(angularSpeed) < minVelocity.get()) {
      angularSpeed = Math.copySign(minVelocity.get(), angularSpeed);
    }

    double linearSpeed = axisProcessor.process(speedSupplier.get());
    drive.drivePercent(linearSpeed - angularSpeed, linearSpeed + angularSpeed);

    // Log data
    Logger.getInstance().recordOutput("ActiveCommands/AutoAim", true);
    Logger.getInstance().recordOutput("AutoAim/ErrorDegrees",
        controller.getPositionError());
    Logger.getInstance().recordOutput("AutoAim/OutputPercent", angularSpeed);
  }

  /**
   * Calculates the rotation which points the robot towards the hub for shooting.
   * 
   * @param position The current robot position
   */
  public static Rotation2d getTargetRotation(Translation2d position) {
    return getTargetRotation(position, FieldConstants.hubCenter, true);
  }

  /**
   * Calculates the rotation which points the robot towards a target.
   * 
   * @param position The current robot position
   * @param target The position to aim towards
   * @param reversed Point the back of the robot towards the target
   */
  public static Rotation2d getTargetRotation(Translation2d position,
      Translation2d target, boolean reversed) {
    Translation2d vehicleToCenter = target.minus(position);
    Rotation2d targetRotation =
        new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
    if (reversed) {
      return targetRotation.plus(Rotation2d.fromDegrees(180));
    } else {
      return targetRotation;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    toleranceTimer.stop();
    if (vision != null) {
      vision.setForceLeds(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime.get());
  }
}
