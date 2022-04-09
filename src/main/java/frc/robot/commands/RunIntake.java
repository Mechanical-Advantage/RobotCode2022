// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.TunableNumber;

public class RunIntake extends CommandBase {
  private static final boolean enableRumbleFirstCargo = true;
  private static final boolean enableRumbleSecondCargo = true;
  private static final TunableNumber rumblePercent =
      new TunableNumber("RunIntake/RumblePercent");
  private static final TunableNumber rumbleDurationSecs =
      new TunableNumber("RunIntake/RumbleDuration");

  private static final TunableNumber forwardsSpeed =
      new TunableNumber("RunIntake/ForwardsSpeed");
  private static final TunableNumber backwardsSpeed =
      new TunableNumber("RunIntake/BackwardsSpeed");

  private final boolean forwards;
  private final Intake intake;
  private final Feeder feeder;
  private final Leds leds;

  private final Consumer<Double> rumbleConsumer;

  private boolean rumbleLastOneTripped = false;
  private boolean rumbleLastTwoTripped = false;
  private boolean rumbleOneActive = false;
  private boolean rumbleTwoActive = false;
  private final Timer rumbleOneTimer = new Timer();
  private final Timer rumbleTwoTimer = new Timer();

  /**
   * Creates a new RunIntake. Runs the intake forwards or backwards, intended for operator controls.
   */
  public RunIntake(boolean forwards, Intake intake, Feeder feeder, Leds leds) {
    this(forwards, intake, feeder, leds, x -> {
    });
  }

  /**
   * Creates a new RunIntake. Runs the intake forwards or backwards, intended for operator controls.
   */
  public RunIntake(boolean forwards, Intake intake, Feeder feeder, Leds leds,
      Consumer<Double> rumbleConsumer) {
    addRequirements(intake);
    this.forwards = forwards;
    this.intake = intake;
    this.feeder = feeder;
    this.leds = leds;
    this.rumbleConsumer = rumbleConsumer;

    rumblePercent.setDefault(0.5);
    rumbleDurationSecs.setDefault(0.2);

    forwardsSpeed.setDefault(1.0);
    backwardsSpeed.setDefault(-1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.requestIntakeForwards(forwards);
    feeder.requestIntakeBackwards(!forwards);
    leds.setIntaking(true);

    rumbleLastOneTripped = feeder.getUpperProxSensor();
    rumbleLastTwoTripped =
        feeder.getLowerProxSensor() && feeder.getUpperProxSensor();
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
      int cargoCount = feeder.getCargoCount();
      if (cargoCount == 2) {
        intake.runPercent(0.0);
      } else {
        intake.runPercent(forwardsSpeed.get());
      }

      boolean rumbleOneTripped = cargoCount >= 1;
      boolean rumbleTwoTripped = cargoCount >= 2;
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
      intake.runPercent(backwardsSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.requestIntakeForwards(false);
    feeder.requestIntakeBackwards(false);
    intake.stop();
    leds.setIntaking(false);
    rumbleConsumer.accept(0.0);
    rumbleOneTimer.stop();
    rumbleTwoTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
