// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import frc.robot.util.TunableNumber;

public class Hood extends SubsystemBase {
  private final TunableNumber resetAngle =
      new TunableNumber("Hood/ResetAngleDegrees");
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
  private final TunableNumber goalTolerance =
      new TunableNumber("Hood/GoalToleranceDegrees");

  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private RobotState robotState;
  private final Timer resetGraceTimer = new Timer();
  private boolean resetComplete = false;
  private boolean resetActive = false;
  private double basePositionRad = 0.0;
  private boolean closedLoop = false;

  /** Creates a new Kicker. */
  public Hood(HoodIO io) {
    this.io = io;
    io.setBrakeMode(true);

    switch (Constants.getRobot()) {
      case ROBOT_2022C:
        resetAngle.setDefault(0.5);
        minAngle.setDefault(6.0);
        maxAngle.setDefault(28.0);
        kP.setDefault(1.0);
        kD.setDefault(0.0);
        maxVelocity.setDefault(225.0);
        maxAcceleration.setDefault(800.0);
        goalTolerance.setDefault(0.5);
        break;
      case ROBOT_SIMBOT:
        minAngle.setDefault(10.0);
        maxAngle.setDefault(80.0);
        kP.setDefault(0.1);
        kD.setDefault(0.0);
        maxVelocity.setDefault(500.0);
        maxAcceleration.setDefault(200.0);
        goalTolerance.setDefault(0.5);
        break;
      default:
        minAngle.setDefault(0.0);
        maxAngle.setDefault(0.0);
        kP.setDefault(0.0);
        kD.setDefault(0.0);
        maxVelocity.setDefault(0.0);
        maxAcceleration.setDefault(0.0);
        goalTolerance.setDefault(0.0);
        break;
    }
  }

  public void setRobotState(RobotState robotState) {
    this.robotState = robotState;
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

    Logger.getInstance().recordOutput("Hood/CurrentAngle", getAngle());

    if (DriverStation.isDisabled()) {
      closedLoop = false;
    }

    if (!resetComplete) {
      if (DriverStation.isEnabled()) {
        // Reset hood
        if (!resetActive) {

        }
      }
    } else {

      // Save hood position to robot state
      robotState.addHoodData(Timer.getFPGATimestamp(), getAngle());

      // Run closed loop control
      if (closedLoop) {
        double volts = controller.calculate(getAngle());
        io.setVoltage(volts);
        Logger.getInstance().recordOutput("Hood/GoalAngle",
            controller.getGoal().position);
        Logger.getInstance().recordOutput("Hood/SetpointAngle",
            controller.getSetpoint().position);
        Logger.getInstance().recordOutput("Hood/AtGoal", atGoal());
      }
    }
  }

  /** Sets the target hood angle in degrees. */
  public void moveToAngle(double angle) {
    if (!closedLoop) {
      controller.reset(getAngle());
    }
    controller.setGoal(MathUtil.clamp(angle, minAngle.get(), maxAngle.get()));
    closedLoop = true;
  }

  /** Sets the target hood angle to the minimum position. */
  public void moveToBottom() {
    moveToAngle(minAngle.get());
  }

  /** Gets the current hood angle in degrees. */
  public double getAngle() {
    Logger.getInstance().recordOutput("Hood/ResetComplete", resetComplete);
    if (resetComplete) {
      return Units.radiansToDegrees(inputs.positionRad - basePositionRad)
          + resetAngle.get();
    } else {
      return minAngle.get();
    }
  }

  /** Returns whether the hood has reached the commanded angle. */
  public boolean atGoal() {
    if (closedLoop) {
      return Math.abs(controller.getGoal().position
          - controller.getSetpoint().position) < goalTolerance.get();
    } else {
      return false;
    }
  }

  /** Resets the current position to the minimum position. */
  public void reset() {
    basePositionRad = inputs.positionRad;
    resetComplete = true;
  }
}
