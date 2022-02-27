// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableNumber;

public class TurnToAngleProfile extends CommandBase {
  private static final double maxVelocityDegreesPerSec = 360.0;
  private static final double maxAccelerationDegreesPerSec2 = 720.0;

  private static final TunableNumber kP =
      new TunableNumber("TurnToAngleProfile/kP");
  private static final TunableNumber kD =
      new TunableNumber("TurnToAngleProfile/kD");

  private final Drive drive;
  private final Rotation2d startRotation;
  private final TrapezoidProfile profile;
  private final Timer timer = new Timer();
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private boolean shouldExit = false;

  /** Creates a new TurnToAngleProfile. */
  public TurnToAngleProfile(Drive drive, Rotation2d startRotation,
      Rotation2d endRotation) {
    addRequirements(drive);
    this.drive = drive;
    this.startRotation = startRotation;

    kP.setDefault(0.008);
    kD.setDefault(0.0);

    Rotation2d relativeEndRotation = endRotation.minus(startRotation);
    profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxVelocityDegreesPerSec,
            maxAccelerationDegreesPerSec2),
        new TrapezoidProfile.State(relativeEndRotation.getDegrees(), 0.0),
        new TrapezoidProfile.State(0.0, 0.0));

    controller.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    controller.reset();
    shouldExit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kP.hasChanged() || kD.hasChanged()) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }

    Rotation2d setpoint =
        Rotation2d.fromDegrees(profile.calculate(timer.get()).position)
            .plus(startRotation);
    double speed = controller.calculate(drive.getRotation().getDegrees(),
        setpoint.getDegrees());
    drive.drivePercent(-speed, speed);

    if (profile.isFinished(timer.get())) {
      shouldExit = true;
    }

    Logger.getInstance().recordOutput("TurnToAngleProfile/Setpoint",
        setpoint.getDegrees());
    Logger.getInstance().recordOutput("TurnToAngleProfile/Speed", speed);
    Logger.getInstance().recordOutput("ActiveCommands/TurnToAngleProfile",
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldExit;
  }

  public double getDuration() {
    return profile.totalTime();
  }
}
