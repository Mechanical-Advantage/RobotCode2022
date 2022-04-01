// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.TunableNumber;

public class RunIntake extends CommandBase {
  private static final boolean enableRumbleFirstCargo = true;
  private static final boolean enableRumbleSecondCargo = true;
  private static final TunableNumber rumblePercent =
      new TunableNumber("RunIntake/RumblePercent");
  private static final TunableNumber rumbleDurationSecs =
      new TunableNumber("RunIntake/RumbleDuration");
  private static final TunableNumber sensorStopDelay =
      new TunableNumber("RunIntake/SensorStopDelay");

  private static final TunableNumber rollerForwardsSpeed =
      new TunableNumber("RunIntake/RollerForwardsSpeed");
  private static final TunableNumber hopperForwardsSpeed =
      new TunableNumber("RunIntake/HopperForwardsSpeed");
  private static final TunableNumber towerForwardsSpeed =
      new TunableNumber("RunIntake/TowerForwardsSpeed");
  private static final TunableNumber kickerForwardsSpeed =
      new TunableNumber("RunIntake/KickerForwardsSpeed");

  private static final TunableNumber rollerBackwardsSpeed =
      new TunableNumber("RunIntake/RollerBackwardsSpeed");
  private static final TunableNumber hopperBackwardsSpeed =
      new TunableNumber("RunIntake/HopperBackwardsSpeed");

  private final boolean forwards;
  private final Intake intake;
  private final Tower tower;
  private final Kicker kicker;
  private final Leds leds;

  private final Consumer<Double> rumbleConsumer;

  private final Timer sensorStopTimer = new Timer();
  private boolean stopTower = false;

  private boolean rumbleLastOneTripped = false;
  private boolean rumbleLastTwoTripped = false;
  private boolean rumbleOneActive = false;
  private boolean rumbleTwoActive = false;
  private final Timer rumbleOneTimer = new Timer();
  private final Timer rumbleTwoTimer = new Timer();

  /**
   * Creates a new RunIntake. Runs the intake forwards or backwards, intended for operator controls.
   */
  public RunIntake(boolean forwards, Intake intake, Tower tower, Kicker kicker,
      Leds leds) {
    this(forwards, intake, tower, kicker, leds, x -> {
    });
  }

  /**
   * Creates a new RunIntake. Runs the intake forwards or backwards, intended for operator controls.
   */
  public RunIntake(boolean forwards, Intake intake, Tower tower, Kicker kicker,
      Leds leds, Consumer<Double> rumbleConsumer) {
    addRequirements(intake);
    if (forwards) {
      addRequirements(tower, kicker);
    }
    this.forwards = forwards;
    this.intake = intake;
    this.tower = tower;
    this.kicker = kicker;
    this.leds = leds;
    this.rumbleConsumer = rumbleConsumer;

    rumblePercent.setDefault(0.5);
    rumbleDurationSecs.setDefault(0.2);
    sensorStopDelay.setDefault(0.1);

    rollerForwardsSpeed.setDefault(0.75);
    hopperForwardsSpeed.setDefault(1.0);
    towerForwardsSpeed.setDefault(0.5);
    kickerForwardsSpeed.setDefault(-0.3);

    rollerBackwardsSpeed.setDefault(-1.0);
    hopperBackwardsSpeed.setDefault(-1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setIntaking(true);
    stopTower = tower.getLowerCargoSensor() && tower.getUpperCargoSensor();
    sensorStopTimer.reset();
    sensorStopTimer.start();

    rumbleLastOneTripped = tower.getUpperCargoSensor();
    rumbleLastTwoTripped =
        tower.getLowerCargoSensor() && tower.getUpperCargoSensor();
    rumbleOneActive = false;
    rumbleTwoActive = false;
    rumbleOneTimer.reset();
    rumbleTwoTimer.reset();
    rumbleOneTimer.start();
    rumbleTwoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/RunIntake", true);

    if (forwards) {
      intake.runRollerPercent(rollerForwardsSpeed.get());
      intake.runHopperPercent(hopperForwardsSpeed.get());

      boolean sensorsTripped =
          tower.getLowerCargoSensor() && tower.getUpperCargoSensor();
      if (!sensorsTripped) {
        sensorStopTimer.reset();
      }
      if (sensorStopDelay.get() == 0.0) {
        stopTower = sensorsTripped;
      } else if (sensorStopTimer.hasElapsed(sensorStopDelay.get())) {
        stopTower = true;
      }
      tower.runPercent(stopTower ? 0.0 : towerForwardsSpeed.get());
      kicker.runPercent(stopTower ? 0.0 : kickerForwardsSpeed.get());

      // Manage rumble
      boolean rumbleOneTripped = tower.getUpperCargoSensor();
      boolean rumbleTwoTripped =
          tower.getUpperCargoSensor() && tower.getLowerCargoSensor();
      if (rumbleOneTripped && !rumbleLastOneTripped && enableRumbleFirstCargo) {
        rumbleOneActive = true;
        rumbleOneTimer.reset();
      }
      if (rumbleTwoTripped && !rumbleLastTwoTripped
          && enableRumbleSecondCargo) {
        rumbleTwoActive = true;
        rumbleTwoTimer.reset();
      }
      if ((rumbleOneActive
          && !rumbleOneTimer.hasElapsed(rumbleDurationSecs.get()))
          || (rumbleTwoActive
              && !rumbleTwoTimer.hasElapsed(rumbleDurationSecs.get()))) {
        rumbleConsumer.accept(rumblePercent.get());
      } else {
        rumbleConsumer.accept(0.0);
      }
      rumbleLastOneTripped = rumbleOneTripped;
      rumbleLastTwoTripped = rumbleTwoTripped;

    } else {
      intake.runRollerPercent(rollerBackwardsSpeed.get());
      intake.runHopperPercent(hopperBackwardsSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(0.0);
    leds.setIntaking(false);
    sensorStopTimer.stop();
    rumbleOneTimer.stop();
    rumbleTwoTimer.stop();
    intake.stop();
    tower.stop();
    kicker.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
