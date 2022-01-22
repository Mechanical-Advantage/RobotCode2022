// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Vision hardware implementation for PhotonVision. */
public class VisionIOPhotonVision implements VisionIO {
  private static final String cameraName = "limelight";

  private final PhotonCamera camera = new PhotonCamera(cameraName);

  private volatile double captureTimestamp = 0.0;
  private volatile double[] cornerX = new double[] {};
  private volatile double[] cornerY = new double[] {};

  public VisionIOPhotonVision() {
    NetworkTableInstance.getDefault()
        .getEntry("/photonvision/" + cameraName + "/latencyMillis")
        .addListener(event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          captureTimestamp = Logger.getInstance().getRealTimestamp()
              - (result.getLatencyMillis() / 1000.0);

          List<Double> cornerXList = new ArrayList<>();
          List<Double> cornerYList = new ArrayList<>();
          for (PhotonTrackedTarget target : result.getTargets()) {
            for (TargetCorner corner : target.getCorners()) {
              cornerXList.add(corner.x);
              cornerYList.add(corner.y);
            }
          }

          cornerX =
              cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
          cornerY =
              cornerYList.stream().mapToDouble(Double::doubleValue).toArray();

        }, EntryListenerFlags.kUpdate);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.cornerX = cornerX;
    inputs.cornerY = cornerY;
  }

  @Override
  public void setLeds(boolean enabled) {
    camera.setLED(enabled ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }
}
