// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SysIdCommand extends CommandBase {
  private final boolean isDriveTrain;
  private BiConsumer<Double, Double> driveTrainSetter;
  private Consumer<Double> mechanismSetter;
  private Supplier<DriveTrainSysIdData> driveTrainGetter;
  private Supplier<MechanismSysIdData> mechanismGetter;

  private double startTime;
  private String data;

  /** Creates a new SysIdCommand for a drive train. */
  public SysIdCommand(
      Subsystem subsystem,
      BiConsumer<Double, Double> driveTrainSetter,
      Supplier<DriveTrainSysIdData> driveTrainGetter) {
    addRequirements(subsystem);
    isDriveTrain = true;
    this.driveTrainSetter = driveTrainSetter;
    this.driveTrainGetter = driveTrainGetter;
  }

  /** Creates a new SysIdCommand for a generic mechanism. */
  public SysIdCommand(
      Subsystem subsystem,
      Consumer<Double> mechanismSetter,
      Supplier<MechanismSysIdData> mechanismGetter) {
    addRequirements(subsystem);
    isDriveTrain = false;
    this.mechanismSetter = mechanismSetter;
    this.mechanismGetter = mechanismGetter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("SysIdOverflow", false);
    SmartDashboard.putString("SysIdTelemetry", "");
    startTime = Timer.getFPGATimestamp();
    data = "";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double timestamp = Timer.getFPGATimestamp();

    // Check if running the correct test
    String test = SmartDashboard.getString("SysIdTest", "");
    boolean correctTest;
    if (isDriveTrain) {
      correctTest = test.equals("Drivetrain") || test.equals("Drivetrain (Angular)");
    } else {
      correctTest = test.equals("Arm") || test.equals("Elevator") || test.equals("Simple");
    }
    SmartDashboard.putBoolean("SysIdWrongMech", !correctTest);

    // Wrong test, prevent movement
    if (!correctTest) {
      if (isDriveTrain) {
        driveTrainSetter.accept(0.0, 0.0);
      } else {
        mechanismSetter.accept(0.0);
      }
      return;
    }

    // Calculate voltage
    String testType = SmartDashboard.getString("SysIdTestType", "");
    double voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
    boolean rotate = SmartDashboard.getBoolean("SysIdRotate", false);
    double baseVoltage;
    switch (testType) {
      case "Quasistatic":
        baseVoltage = voltageCommand * (timestamp - startTime);
        break;
      case "Dynamic":
        baseVoltage = voltageCommand;
        break;
      default:
        baseVoltage = 0.0;
        break;
    }
    double primaryVoltage = baseVoltage * (rotate ? -1 : 1);
    double secondaryVoltage = baseVoltage;

    // Set output and get new data
    if (isDriveTrain) {
      driveTrainSetter.accept(primaryVoltage, secondaryVoltage);
      DriveTrainSysIdData subsystemData = driveTrainGetter.get();

      data += Double.toString(timestamp) + ",";
      data += Double.toString(primaryVoltage) + ",";
      data += Double.toString(secondaryVoltage) + ",";
      data += Double.toString(subsystemData.leftPosRad / (2 * Math.PI)) + ",";
      data += Double.toString(subsystemData.rightPosRad / (2 * Math.PI)) + ",";
      data += Double.toString(subsystemData.leftVelRadPerSec / (2 * Math.PI)) + ",";
      data += Double.toString(subsystemData.rightVelRadPerSec / (2 * Math.PI)) + ",";
      data += Double.toString(subsystemData.gyroPosRad) + ",";
      data += Double.toString(subsystemData.gyroVelRadPerSec) + ",";
    } else {
      mechanismSetter.accept(primaryVoltage);
      MechanismSysIdData subsystemData = mechanismGetter.get();

      data += Double.toString(timestamp) + ",";
      data += Double.toString(primaryVoltage) + ",";
      data += Double.toString(subsystemData.posRad / (2 * Math.PI)) + ",";
      data += Double.toString(subsystemData.velRadPerSec / (2 * Math.PI)) + ",";
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (data.length() > 0) {
      SmartDashboard.putString("SysIdTelemetry", data.substring(0, data.length() - 1));
      System.out.println(
          "Saved " + Long.toString(Math.round(data.length() / 1024.0)) + " KB of data.");
    } else {
      System.out.println("No data to save. Something's gone wrong here...");
    }
    if (isDriveTrain) {
      driveTrainSetter.accept(0.0, 0.0);
    } else {
      mechanismSetter.accept(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** SysId data for a drivetrain, returned by the subsystem. */
  public static class DriveTrainSysIdData {
    public final double leftPosRad;
    public final double rightPosRad;
    public final double leftVelRadPerSec;
    public final double rightVelRadPerSec;
    public final double gyroPosRad;
    public final double gyroVelRadPerSec;

    /**
     * Creates a new DriveTrainSysIdData.
     *
     * @param leftPosRad Left position (radians)
     * @param rightPosRad Right position (radians)
     * @param leftVelRadPerSec Left velocity (radians per second)
     * @param rightVelRadPerSec Right velocity (radians per second)
     * @param gyroPosRad Gyro position (radians)
     * @param gyroVelRadPerSec Gyro position (radians per second)
     */
    public DriveTrainSysIdData(
        double leftPosRad,
        double rightPosRad,
        double leftVelRadPerSec,
        double rightVelRadPerSec,
        double gyroPosRad,
        double gyroVelRadPerSec) {
      this.leftPosRad = leftPosRad;
      this.rightPosRad = rightPosRad;
      this.leftVelRadPerSec = leftVelRadPerSec;
      this.rightVelRadPerSec = rightVelRadPerSec;
      this.gyroPosRad = gyroPosRad;
      this.gyroVelRadPerSec = gyroVelRadPerSec;
    }
  }

  /** SysId data for a generic mechanism, returned by the subsystem. */
  public static class MechanismSysIdData {
    public final double posRad;
    public final double velRadPerSec;

    /**
     * Creates a new MechanismSysIdData.
     *
     * @param posRad Position (radians)
     * @param velRadPerSec Velocity (radians per second)
     */
    public MechanismSysIdData(double posRad, double velRadPerSec) {
      this.posRad = posRad;
      this.velRadPerSec = velRadPerSec;
    }
  }
}
