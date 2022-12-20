// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Pneumatics subsystem hardware interface. */
public interface PneumaticsIO {
  /** Contains all of the input data received from hardware. */
  public static class PneumaticsIOInputs implements LoggableInputs {
    public double pressurePsi = 0.0;
    public boolean compressorActive = false;
    public double compressorCurrentAmps = 0.0;

    public void toLog(LogTable table) {
      table.put("PressurePsi", pressurePsi);
      table.put("CompressorActive", compressorActive);
      table.put("CompressorCurrentAmps", compressorCurrentAmps);
    }

    public void fromLog(LogTable table) {
      pressurePsi = table.getDouble("PressurePsi", pressurePsi);
      compressorActive = table.getBoolean("CompressorActive", compressorActive);
      compressorCurrentAmps = table.getDouble("CompressorCurrentAmps", compressorCurrentAmps);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PneumaticsIOInputs inputs) {}

  /** Updates the compressor threshold */
  public default void useLowClosedLoopThresholds(boolean useLow) {}
}
