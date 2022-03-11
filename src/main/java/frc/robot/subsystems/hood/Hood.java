// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import frc.robot.util.TunableNumber;

public class Hood extends SubsystemBase {
  private final TunableNumber minAngle =
      new TunableNumber("Hood/MinAngleDegrees");
  private final TunableNumber maxAngle =
      new TunableNumber("Hood/MaxAngleDegres");
  private final TunableNumber kP = new TunableNumber("Hood/kP");
  private final TunableNumber kD = new TunableNumber("Hood/kD");
  private final TunableNumber maxVelocity =
      new TunableNumber("Hood/MaxVelocity");
  private final TunableNumber maxAcceleration =
      new TunableNumber("Hood/MaxAcceleration");

  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double basePositionRad = 0.0;
  private boolean closedLoop = false;

  /** Creates a new Kicker. */
  public Hood(HoodIO io) {
    this.io = io;
    io.setBrakeMode(true);

    minAngle.setDefault(10.0);
    maxAngle.setDefault(70.0);
    kP.setDefault(0.0);
    kD.setDefault(0.0);
    maxVelocity.setDefault(0.0);
    maxAcceleration.setDefault(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Hood", inputs);

    if (kP.hasChanged()) {
      controller.setP(kP.get());
    }
    if (kD.hasChanged()) {
      controller.setP(kD.get());
    }
    if (maxVelocity.hasChanged() || maxAcceleration.hasChanged()) {
      controller.setConstraints(new TrapezoidProfile.Constraints(
          maxVelocity.get(), maxAcceleration.get()));
    }

    if (closedLoop) {
      double volts = controller.calculate(getAngle());
      io.setVoltage(volts);
    }
  }

  /** Sets the target hood angle in degrees. */
  public void setAngle(double angle) {
    if (!closedLoop) {
      controller.reset(getAngle());
    }
    controller.setGoal(angle);
    closedLoop = true;
  }

  /** Gets the current hood angle in degrees. */
  public double getAngle() {
    return Units.radiansToDegrees(inputs.positionRad - basePositionRad)
        + minAngle.get();
  }

  /** Returns whether the hood has reached the commanded angle. */
  public boolean atGoal() {
    return controller.getGoal().equals(controller.getSetpoint());
  }

  /** Runs open loop at the specified percent (for reseting) */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
    closedLoop = false;
  }

  /** Returns the current velocity in degrees per second. */
  public double getVelocityDegreesPerSec() {
    return Units.radiansToDegrees(inputs.velocityRadPerSec);
  }

  /** Resets the current position to the minimum position. */
  public void reset() {
    basePositionRad = inputs.positionRad;
  }
}
