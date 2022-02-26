// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Vision hardware implementation for a Limelight. */
public class VisionIOLimelight implements VisionIO {
  private double captureTimestamp = 0.0;
  private double[] cornerX = new double[] {};
  private double[] cornerY = new double[] {};

  private final NetworkTableEntry ledEntry = NetworkTableInstance.getDefault()
      .getTable("limelight").getEntry("ledMode");
  private final NetworkTableEntry validEntry =
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry latencyEntry =
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
  private final NetworkTableEntry dataEntry = NetworkTableInstance.getDefault()
      .getTable("limelight").getEntry("tcornxy");

  public VisionIOLimelight() {
    latencyEntry.addListener(event -> {
      double timestamp = Logger.getInstance().getRealTimestamp()
          - (latencyEntry.getDouble(0.0) / 1000.0);

      List<Double> cornerXList = new ArrayList<>();
      List<Double> cornerYList = new ArrayList<>();
      if (validEntry.getDouble(0.0) == 1.0) {
        boolean isX = true;
        for (double coordinate : dataEntry.getDoubleArray(new double[] {})) {
          if (isX) {
            cornerXList.add(coordinate);
          } else {
            cornerYList.add(coordinate);
          }
          isX = !isX;
        }
      }

      synchronized (VisionIOLimelight.this) {
        captureTimestamp = timestamp;
        cornerX =
            cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
        cornerY =
            cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
      }

    }, EntryListenerFlags.kUpdate);
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.cornerX = cornerX;
    inputs.cornerY = cornerY;
  }

  @Override
  public void setLeds(boolean enabled) {
    ledEntry.forceSetDouble(enabled ? 3.0 : 1.0);
  }
}
