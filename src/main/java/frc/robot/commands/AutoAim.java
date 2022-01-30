// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.TunableNumber;

public class AutoAim extends CommandBase {
  private final Drive drive;
  private final Vision vision;

  private final PIDController controller;
  private final Timer toleranceTimer = new Timer();

  private final TunableNumber kP = new TunableNumber("AutoAim/kP");
  private final TunableNumber kI = new TunableNumber("AutoAim/kI");
  private final TunableNumber kD = new TunableNumber("AutoAim/kD");
  private final TunableNumber integralMaxError =
      new TunableNumber("AutoAim/IntegralMaxError");
  private final TunableNumber minVelocity =
      new TunableNumber("AutoAim/MinVelocity");
  private final TunableNumber toleranceDegrees =
      new TunableNumber("AutoAim/ToleranceDegrees");
  private final TunableNumber toleranceTime =
      new TunableNumber("AutoAim/ToleranceTime");

  /** Creates a new AutoAim. Points towards the center of the field using odometry data. */
  public AutoAim(Drive drive, Vision vision) {
    addRequirements(drive, vision);
    this.drive = drive;
    this.vision = vision;

    switch (Constants.getRobot()) {
      case ROBOT_2020:
        kP.setDefault(0.018);
        kI.setDefault(0.005);
        kD.setDefault(0.001);
        integralMaxError.setDefault(4);
        minVelocity.setDefault(0.02);
        toleranceDegrees.setDefault(1.5);
        toleranceTime.setDefault(0.1);
        break;
      default:
        break;
    }

    controller = new PIDController(kP.get(), kI.get(), kD.get());
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
    if (kD.hasChanged()) {
      controller.setD(kD.get());
    }
    if (toleranceDegrees.hasChanged()) {
      controller.setTolerance(toleranceDegrees.get());
    }

    // Update setpoint
    Translation2d vehicleToCenter =
        FieldConstants.hubCenter.minus(drive.getPose().getTranslation());
    Rotation2d targetRotation =
        new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
    controller.setSetpoint(targetRotation.getDegrees());

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
    double output = controller.calculate(drive.getRotation().getDegrees());
    if (Math.abs(output) < minVelocity.get()) {
      output = Math.copySign(minVelocity.get(), output);
    }
    drive.drivePercent(output * -1, output);
    Logger.getInstance().recordOutput("AutoAim/ErrorDegrees",
        controller.getPositionError());
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
