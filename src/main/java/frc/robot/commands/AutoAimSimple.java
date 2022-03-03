// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.DriveWithJoysticks.AxisProcessor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TunableNumber;

public class AutoAimSimple extends CommandBase {
  private static final TunableNumber kP = new TunableNumber("AutoAimSimple/kP");
  private static final TunableNumber kI = new TunableNumber("AutoAimSimple/kI");
  private static final TunableNumber kD = new TunableNumber("AutoAimSimple/kD");
  private static final TunableNumber toleranceDegrees =
      new TunableNumber("AutoAimSimple/ToleranceDegrees");
  private static final TunableNumber toleranceTime =
      new TunableNumber("AutoAimSimple/ToleranceTime");

  private final Drive drive;
  private final Vision vision;
  private final Supplier<Double> speedSupplier;

  private final PIDController controller;
  private final Timer toleranceTimer = new Timer();
  private final AxisProcessor axisProcessor = new AxisProcessor();

  public AutoAimSimple(Drive drive, Vision vision) {
    this(drive, vision, () -> 0.0);
  }

  /**
   * Creates a new AutoAimSimple. Points towards the center of the field using simple vision data.
   */
  public AutoAimSimple(Drive drive, Vision vision,
      Supplier<Double> speedSupplie) {
    addRequirements(drive, vision);
    this.drive = drive;
    this.vision = vision;
    this.speedSupplier = speedSupplie;

    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        kP.setDefault(0.005);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceDegrees.setDefault(3.0);
        toleranceTime.setDefault(0.3);
        break;
      default:
        kP.setDefault(0.0);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        toleranceDegrees.setDefault(0.0);
        toleranceTime.setDefault(0.0);
        break;
    }

    controller = new PIDController(kP.get(), kI.get(), kD.get(),
        Constants.loopPeriodSecs);
    controller.setSetpoint(0.0);
    controller.setTolerance(toleranceDegrees.get());
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    toleranceTimer.reset();
    toleranceTimer.start();
    vision.setForceLeds(true);
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

    // Check if in tolerance
    if (!controller.atSetpoint()) {
      toleranceTimer.reset();
    }

    // Calculate angular speed
    double angularSpeed = 0.0;
    if (vision.getSimpleTargetValid()) {
      angularSpeed = controller.calculate(vision.getSimpleTargetAngle());
    } else {
      controller.reset();
    }

    // Run at calculated speed
    double linearSpeed = axisProcessor.process(speedSupplier.get());
    drive.drivePercent(linearSpeed - angularSpeed, linearSpeed + angularSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    toleranceTimer.stop();
    vision.setForceLeds(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime.get());
  }
}
