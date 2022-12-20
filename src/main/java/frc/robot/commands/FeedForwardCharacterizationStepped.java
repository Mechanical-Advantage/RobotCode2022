// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FeedForwardCharacterizationStepped extends CommandBase {
  private static final double startDelaySecs = 2.0; // How long to wait when starting
  private static final double stepLengthSecs = 5.0; // How many seconds for each full step
  private static final double stepIgnoreSecs = 3.0; // How many seconds of data to ignore after
  // increasing voltage
  private static final double initialVolts = 1.0; // The voltage of the first step
  private static final double stepSizeVolts = 0.2; // How much to increase the voltage each step

  private final boolean forwards;
  private final boolean isDrive;

  private final FeedForwardCharacterizationData dataPrimary;
  private final FeedForwardCharacterizationData dataSecondary;
  private final Consumer<Double> voltageConsumerSimple;
  private final BiConsumer<Double, Double> voltageConsumerDrive;
  private final Supplier<Double> velocitySupplierPrimary;
  private final Supplier<Double> velocitySupplierSecondary;
  private final Supplier<Double> voltageSupplierPrimary;
  private final Supplier<Double> voltageSupplierSecondary;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterizationStepped for a drive. */
  public FeedForwardCharacterizationStepped(
      Subsystem drive,
      boolean forwards,
      FeedForwardCharacterizationData leftData,
      FeedForwardCharacterizationData rightData,
      BiConsumer<Double, Double> voltageConsumer,
      Supplier<Double> leftVelocitySupplier,
      Supplier<Double> rightVelocitySupplier,
      Supplier<Double> leftVoltageSupplier,
      Supplier<Double> rightVoltageSupplier) {
    addRequirements(drive);
    this.forwards = forwards;
    this.isDrive = true;
    this.dataPrimary = leftData;
    this.dataSecondary = rightData;
    this.voltageConsumerSimple = null;
    this.voltageConsumerDrive = voltageConsumer;
    this.velocitySupplierPrimary = leftVelocitySupplier;
    this.velocitySupplierSecondary = rightVelocitySupplier;
    this.voltageSupplierPrimary = leftVoltageSupplier;
    this.voltageSupplierSecondary = rightVoltageSupplier;
  }

  /** Creates a new FeedForwardCharacterizationStepped for a simple subsystem. */
  public FeedForwardCharacterizationStepped(
      Subsystem subsystem,
      boolean forwards,
      FeedForwardCharacterizationData data,
      Consumer<Double> voltageConsumer,
      Supplier<Double> velocitySupplier,
      Supplier<Double> voltageSupplier) {
    addRequirements(subsystem);
    this.forwards = forwards;
    this.isDrive = false;
    this.dataPrimary = data;
    this.dataSecondary = null;
    this.voltageConsumerSimple = voltageConsumer;
    this.voltageConsumerDrive = null;
    this.velocitySupplierPrimary = velocitySupplier;
    this.velocitySupplierSecondary = null;
    this.voltageSupplierPrimary = voltageSupplier;
    this.voltageSupplierSecondary = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelaySecs) {
      if (isDrive) {
        voltageConsumerDrive.accept(0.0, 0.0);
      } else {
        voltageConsumerSimple.accept(0.0);
      }
    } else {
      int stepIndex = (int) Math.floor((timer.get() - startDelaySecs) / stepLengthSecs);
      double voltage = initialVolts + (stepIndex * stepSizeVolts) * (forwards ? 1 : -1);
      if (isDrive) {
        voltageConsumerDrive.accept(voltage, voltage);
      } else {
        voltageConsumerSimple.accept(voltage);
      }

      if ((timer.get() - startDelaySecs) % stepLengthSecs > stepIgnoreSecs) {
        dataPrimary.add(velocitySupplierPrimary.get(), voltageSupplierPrimary.get());
        if (isDrive) {
          dataSecondary.add(velocitySupplierSecondary.get(), voltageSupplierSecondary.get());
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isDrive) {
      voltageConsumerDrive.accept(0.0, 0.0);
    } else {
      voltageConsumerSimple.accept(0.0);
    }
    timer.stop();
    dataPrimary.print();
    if (isDrive) {
      dataSecondary.print();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
