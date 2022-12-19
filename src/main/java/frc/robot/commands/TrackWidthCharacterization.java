// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class TrackWidthCharacterization extends CommandBase {
  private static final double rampRateVoltsPerSec = 0.1;
  private static final double maxVolts = 2.0;

  private final BiConsumer<Double, Double> voltageConsumer;
  private final Supplier<Double> leftPositionSupplier;
  private final Supplier<Double> rightPositionSupplier;
  private final Supplier<Double> gyroPositionSupplier;

  private Timer timer = new Timer();
  private double basePositionLeft;
  private double basePositionRight;
  private double baseGyroPosition;

  /** Creates a new TrackWidthCharacterization. */
  public TrackWidthCharacterization(
      Subsystem drive,
      BiConsumer<Double, Double> voltageConsumer,
      Supplier<Double> leftPositionSupplier,
      Supplier<Double> rightPositionSupplier,
      Supplier<Double> gyroPositionSupplier) {
    addRequirements(drive);
    this.voltageConsumer = voltageConsumer;
    this.leftPositionSupplier = leftPositionSupplier;
    this.rightPositionSupplier = rightPositionSupplier;
    this.gyroPositionSupplier = gyroPositionSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    basePositionLeft = leftPositionSupplier.get();
    basePositionRight = rightPositionSupplier.get();
    baseGyroPosition = gyroPositionSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = timer.get() * rampRateVoltsPerSec;
    voltage = voltage > maxVolts ? maxVolts : voltage;
    voltageConsumer.accept(voltage, voltage * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0, 0.0);
    timer.stop();

    double leftDistance = Math.abs(leftPositionSupplier.get() - basePositionLeft);
    double rightDistance = Math.abs(rightPositionSupplier.get() - basePositionRight);
    double gyroDistance = Math.abs(gyroPositionSupplier.get() - baseGyroPosition);
    double trackWidth = (leftDistance + rightDistance) / gyroDistance;

    System.out.println(String.format("Track width=%.5f", trackWidth));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
