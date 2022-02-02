// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.BuildConstants;

/** Determines whether to burn SparkMAX configs to flash. */
public class SparkMAXBurnManager {
  private static final String buildDateFile = "/home/lvuser/build-date.txt";
  private static boolean shouldBurn = false;

  private SparkMAXBurnManager() {}

  public static boolean shouldBurn() {
    return shouldBurn;
  }

  public static void update() {
    if (RobotBase.isSimulation()) {
      System.out.println("[SparkMAXBurnManager] Running in simulation, will not burn SparkMAX flash");
      shouldBurn = false;
      return;
    }

    File file = new File(buildDateFile);
    if (!file.exists()) {

      // No build date file, burn flash
      shouldBurn = true;
    } else {

      // Read previous build date
      String previousBuildDate = "";
      try {
        previousBuildDate =
            new String(Files.readAllBytes(Paths.get(buildDateFile)),
                StandardCharsets.UTF_8);
      } catch (IOException e) {
        e.printStackTrace();
      }

      shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
    }

    try {
      FileWriter fileWriter = new FileWriter(buildDateFile);
      fileWriter.write(BuildConstants.BUILD_DATE);
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (shouldBurn) {
      System.out.println(
          "[SparkMAXBurnManager] Build date changed, burning SparkMAX flash");
    } else {
      System.out.println(
          "[SparkMAXBurnManager] Build date unchanged, will not burn SparkMAX flash");
    }
  }
}

