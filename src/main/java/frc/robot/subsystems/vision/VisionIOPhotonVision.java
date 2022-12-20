// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Vision hardware implementation for PhotonVision. */
public class VisionIOPhotonVision implements VisionIO {
  private static final String cameraName = "limelight";
  private final PhotonCamera camera = new PhotonCamera(cameraName);

  private double captureTimestamp = 0.0;
  private double[] cornerX = new double[] {};
  private double[] cornerY = new double[] {};

  public VisionIOPhotonVision() {
    NetworkTableInstance.getDefault()
        .getEntry("/photonvision/" + cameraName + "/latencyMillis")
        .addListener(
            event -> {
              PhotonPipelineResult result = camera.getLatestResult();
              double timestamp =
                  Logger.getInstance().getRealTimestamp() - (result.getLatencyMillis() / 1000.0);

              List<Double> cornerXList = new ArrayList<>();
              List<Double> cornerYList = new ArrayList<>();
              for (PhotonTrackedTarget target : result.getTargets()) {
                for (TargetCorner corner : target.getCorners()) {
                  cornerXList.add(corner.x);
                  cornerYList.add(corner.y);
                }
              }

              synchronized (VisionIOPhotonVision.this) {
                captureTimestamp = timestamp;
                cornerX = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
                cornerY = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
              }
            },
            EntryListenerFlags.kUpdate);
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.cornerX = cornerX;
    inputs.cornerY = cornerY;
  }

  @Override
  public void setLeds(boolean enabled) {
    camera.setLED(enabled ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }
}
