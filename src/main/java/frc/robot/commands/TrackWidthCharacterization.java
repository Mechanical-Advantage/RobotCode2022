// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TrackWidthCharacterization extends CommandBase {
  private static final double spinVoltage = 2.0;

  private final BiConsumer<Double, Double> voltageConsumer;
  private final Supplier<Double> leftPositionSupplier;
  private final Supplier<Double> rightPositionSupplier;
  private final Supplier<Double> gyroPositionSupplier;

  private double basePositionLeft;
  private double basePositionRight;
  private double baseGyroPosition;

  /** Creates a new TrackWidthCharacterization. */
  public TrackWidthCharacterization(Subsystem drive,
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
    voltageConsumer.accept(spinVoltage, spinVoltage * -1);
    basePositionLeft = leftPositionSupplier.get();
    basePositionRight = rightPositionSupplier.get();
    baseGyroPosition = gyroPositionSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0, 0.0);

    double leftDistance =
        Math.abs(leftPositionSupplier.get() - basePositionLeft);
    double rightDistance =
        Math.abs(rightPositionSupplier.get() - basePositionRight);
    double gyroDistance =
        Math.abs(gyroPositionSupplier.get() - baseGyroPosition);
    double trackWidth = (leftDistance + rightDistance) / gyroDistance;

    System.out.println(String.format("Track width=%.5f", trackWidth));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
