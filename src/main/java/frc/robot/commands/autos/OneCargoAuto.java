// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.PrepareShooterAuto;
import frc.robot.commands.Shoot;
import frc.robot.commands.WaitForVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;

public class OneCargoAuto extends SequentialCommandGroup {
  public static final double shootDurationSecs = 1.5;

  /** Creates a new OneCargoAuto. Aims towards the target, shoots, and taxis. */
  public OneCargoAuto(boolean longTaxi, RobotState robotState, Drive drive,
      Vision vision, Flywheels flywheels, Hood hood, Feeder feeder, Leds leds) {
    addCommands(
        deadline(
            sequence(new WaitForVision(robotState),
                new AutoAim(drive, robotState, vision, null, null),
                new WaitUntilCommand(
                    () -> flywheels.atSetpoint() && hood.atGoal()),
                new Shoot(feeder, leds).withTimeout(shootDurationSecs)),
            new PrepareShooterAuto(flywheels, hood, feeder, robotState)),
        new Taxi(drive, longTaxi));
  }
}
