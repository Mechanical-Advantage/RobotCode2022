// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.PneumaticsIO.PneumaticsIOInputs;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Pneumatics extends SubsystemBase {
  public static final int revModuleID = 60; // CAN ID for pneumatics hub
  private static final int normalAveragingTaps = 20;
  private static final int compressorAveragingTaps = 150;
  private static final int maxTapRemoval = 2; // Per cycle, smoothss data when compressor stops
  private static final double compressorRatePsiPerSec = 3.0;

  private final PneumaticsIO io;
  private final PneumaticsIOInputs inputs = new PneumaticsIOInputs();

  private final List<Double> filterData = new ArrayList<>();
  private boolean lastCompressorActive = false;
  private int compressorStateCycles = 0;
  private double pressureSmoothedPsi = 0.0;

  private Timer noPressureTimer = new Timer();
  private Timer compressorEnabledTimer = new Timer();
  private Alert dumpValveAlert = new Alert(
      "Cannot build pressure. Is the dump value open?", AlertType.WARNING);

  /** Creates a new Pneumatics. */
  public Pneumatics(PneumaticsIO io) {
    this.io = io;
    noPressureTimer.start();
    compressorEnabledTimer.start();
  }

  public double getPressure() {
    return pressureSmoothedPsi;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Pneumatics", inputs);

    // Run averaging filter
    filterData.add(inputs.pressurePsi < 0.0 ? 0.0 : inputs.pressurePsi);
    int averagingTaps =
        inputs.compressorActive ? compressorAveragingTaps : normalAveragingTaps;
    int i = 0;
    while (filterData.size() > averagingTaps && i < maxTapRemoval) {
      i++;
      filterData.remove(0);
    }
    double averagePsi = filterData.stream().mapToDouble(a -> a)
        .summaryStatistics().getAverage();

    // Compensate for latency when compressor is active
    if (inputs.compressorActive != lastCompressorActive) {
      compressorStateCycles = 0;
      lastCompressorActive = inputs.compressorActive;
    } else {
      compressorStateCycles++;
    }
    int latencyCycles = filterData.size() / 2;
    if (inputs.compressorActive) {
      latencyCycles =
          compressorStateCycles < latencyCycles ? compressorStateCycles
              : latencyCycles;
    } else {
      latencyCycles = compressorStateCycles < latencyCycles
          ? latencyCycles - compressorStateCycles
          : 0;
    }
    double compensationPsi =
        latencyCycles * compressorRatePsiPerSec * Constants.loopPeriodSecs;
    pressureSmoothedPsi = averagePsi + compensationPsi;

    // Log pressure
    Logger.getInstance().recordOutput("PressurePsi", pressureSmoothedPsi);
    SmartDashboard.putNumber("Pressure", pressureSmoothedPsi);

    // Detect if dump value is open
    if (inputs.pressurePsi > 1) {
      noPressureTimer.reset();
    }
    if (!inputs.compressorActive) {
      compressorEnabledTimer.reset();
    }
    dumpValveAlert.set(
        noPressureTimer.hasElapsed(5) && compressorEnabledTimer.hasElapsed(5));
  }
}
