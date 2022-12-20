// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.pneumatics.PneumaticsIO.PneumaticsIOInputs;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pneumatics extends SubsystemBase {
  public static final int revModuleID = 60; // CAN ID for pneumatics hub
  private static final int normalAveragingTaps = 25;
  private static final int compressorAveragingTaps = 50;
  private static final double compressorRatePsiPerSec = 1.5;

  private final PneumaticsIO io;
  private final PneumaticsIOInputs inputs = new PneumaticsIOInputs();

  private final List<Double> filterData = new ArrayList<>();
  private double pressureSmoothedPsi = 0.0;

  private double lastPressurePsi = 0.0;
  private boolean lastPressureIncreasing = false;
  private double compressorMaxPoint = 0.0;
  private double compressorMinPoint = 0.0;

  private Supplier<Boolean> climbModeOverride = () -> false;

  private Timer noPressureTimer = new Timer();
  private Timer compressorEnabledTimer = new Timer();
  private Alert dumpValveAlert =
      new Alert("Cannot build pressure. Is the dump value open?", AlertType.WARNING);

  /** Creates a new Pneumatics. */
  public Pneumatics(PneumaticsIO io) {
    this.io = io;
    noPressureTimer.start();
    compressorEnabledTimer.start();
  }

  public double getPressure() {
    return pressureSmoothedPsi;
  }

  public void setSupplier(Supplier<Boolean> climbModeOverride) {
    this.climbModeOverride = climbModeOverride;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Pneumatics", inputs);

    // Set the compressor threshold depending on whether robot is in climb mode
    io.useLowClosedLoopThresholds(climbModeOverride.get());

    // Calculate input pressure for averaging filter
    double limitedPressure = inputs.pressurePsi < 0.0 ? 0.0 : inputs.pressurePsi;
    double processedPressure = 0.0;
    if (inputs.compressorActive) {

      // When compressor is active, average the most recent min and max points
      if (limitedPressure != lastPressurePsi) {
        boolean increasing = limitedPressure > lastPressurePsi;
        if (increasing != lastPressureIncreasing) {
          if (increasing) {
            compressorMinPoint = lastPressurePsi;
          } else {
            compressorMaxPoint = lastPressurePsi;
          }
        }
        lastPressurePsi = limitedPressure;
        lastPressureIncreasing = increasing;
      }
      processedPressure = (compressorMinPoint + compressorMaxPoint) / 2.0;

      // Apply latency compensation
      processedPressure +=
          (compressorAveragingTaps / 2.0) * Constants.loopPeriodSecs * compressorRatePsiPerSec;
    } else {

      // When compressor is inactive, reset min/max status and use normal pressure
      lastPressurePsi = limitedPressure;
      lastPressureIncreasing = false;
      compressorMaxPoint = limitedPressure;
      compressorMinPoint = limitedPressure;
      processedPressure = limitedPressure;
    }

    // Run averaging filter
    filterData.add(processedPressure);
    while (filterData.size()
        > (inputs.compressorActive ? compressorAveragingTaps : normalAveragingTaps)) {
      filterData.remove(0);
    }
    pressureSmoothedPsi = filterData.stream().mapToDouble(a -> a).summaryStatistics().getAverage();

    // Log pressure
    Logger.getInstance().recordOutput("PressurePsi", pressureSmoothedPsi);
    SmartDashboard.putNumber("Pressure", pressureSmoothedPsi);

    // Detect if dump value is open
    if (inputs.pressurePsi > 3) {
      noPressureTimer.reset();
    }
    if (!inputs.compressorActive) {
      compressorEnabledTimer.reset();
    }
    dumpValveAlert.set(noPressureTimer.hasElapsed(5) && compressorEnabledTimer.hasElapsed(5));
  }
}
