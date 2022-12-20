// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class TurnToAngle extends CommandBase {
  private final Drive drive;
  private final RobotState robotState;
  private final Rotation2d setpoint;
  private final boolean relative;

  private final PIDController controller;
  private final Timer toleranceTimer = new Timer();

  private static final TunableNumber kP = new TunableNumber("TurnToAngle/kP");
  private static final TunableNumber kI = new TunableNumber("TurnToAngle/kI");
  private static final TunableNumber kD = new TunableNumber("TurnToAngle/kD");
  private static final TunableNumber integralMaxError =
      new TunableNumber("TurnToAngle/IntegralMaxError");
  private static final TunableNumber minVelocity = new TunableNumber("TurnToAngle/MinVelocity");
  private static final TunableNumber toleranceDegrees =
      new TunableNumber("TurnToAngle/ToleranceDegrees");
  private static final TunableNumber toleranceTime = new TunableNumber("TurnToAngle/ToleranceTime");

  /** Creates a new TurnToAngle. Turns to the specified rotation. */
  public TurnToAngle(Drive drive, RobotState robotState, Rotation2d setpoint, boolean relative) {
    addRequirements(drive);
    this.drive = drive;
    this.robotState = robotState;
    this.setpoint = setpoint;
    this.relative = relative;

    switch (Constants.getRobot()) {
      case ROBOT_2022C:
      case ROBOT_2022P:
        kP.setDefault(0.0035);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        integralMaxError.setDefault(0.0);
        minVelocity.setDefault(0.0);
        toleranceDegrees.setDefault(1.0);
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

    controller = new PIDController(kP.get(), kI.get(), kD.get(), Constants.loopPeriodSecs);
    controller.setTolerance(toleranceDegrees.get());
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    toleranceTimer.reset();
    toleranceTimer.start();

    if (relative) {
      controller.setSetpoint(robotState.getLatestRotation().rotateBy(setpoint).getDegrees());
    } else {
      controller.setSetpoint(setpoint.getDegrees());
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
    double angularSpeed = controller.calculate(robotState.getLatestRotation().getDegrees());
    if (Math.abs(angularSpeed) < minVelocity.get()) {
      angularSpeed = Math.copySign(minVelocity.get(), angularSpeed);
    }
    drive.drivePercent(-angularSpeed, angularSpeed);

    // Log data
    Logger.getInstance().recordOutput("ActiveCommands/TurnToAngle", true);
    Logger.getInstance().recordOutput("TurnToAngle/ErrorDegrees", controller.getPositionError());
    Logger.getInstance().recordOutput("TurnToAngle/OutputPercent", angularSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    toleranceTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime.get());
  }
}
