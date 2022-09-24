// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.duck.Duck;
import frc.robot.subsystems.duck.Duck.DuckSound;

public class IdleDuck extends CommandBase {
  private static DuckSound quackSound = DuckSound.QUACK_1;
  private static double minSpeedMetersPerSec = Units.inchesToMeters(2.0);
  private static double maxSpeedMetersPerSec = Units.inchesToMeters(150.0);
  private static double minAccelG = 0.1;
  private static double maxAccelG = 1.5;
  private static double minQuackPeriodSecs = 0.25;
  private static double maxQuackPeriodSecs = 1.25;

  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  private final Duck duck;
  private final Drive drive;

  private final Timer timer = new Timer();

  /**
   * Creates a new IdleDuck. Automatically plays quacking sounds and spins the duck based on the
   * drive speed.
   */
  public IdleDuck(Duck duck, Drive drive) {
    addRequirements(duck);
    this.duck = duck;
    this.drive = drive;
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/IdleDuck", true);

    double speedScalar = 0;

    if (DriverStation.isEnabled()) { // Use drive speed
      double averageSpeedMetersPerSec =
          (Math.abs(drive.getLeftVelocityMetersPerSec())
              + Math.abs(drive.getRightVelocityMetersPerSec())) / 2.0;
      speedScalar =
          MathUtil.clamp((averageSpeedMetersPerSec - minSpeedMetersPerSec)
              / (maxSpeedMetersPerSec - minSpeedMetersPerSec), 0.0, 1.0);
      duck.runPercent(0); // Do not spin the duck
    } else { // Use accelerometer
      double totalG =
          Math.abs(accelerometer.getX()) + Math.abs(accelerometer.getY());
      Logger.getInstance().recordOutput("IdleDuck/TotalG", totalG);
      speedScalar = MathUtil
          .clamp((totalG - minAccelG) / (maxAccelG - minAccelG), 0.0, 1.0);
    }

    if (speedScalar > 0) {
      double quackPeriod = maxQuackPeriodSecs
          - ((maxQuackPeriodSecs - minQuackPeriodSecs) * speedScalar);
      if (timer.hasElapsed(quackPeriod)) {
        duck.playSound(quackSound);
        timer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    duck.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
