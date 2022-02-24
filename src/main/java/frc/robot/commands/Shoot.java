// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.tower.Tower;
import frc.robot.util.LedSelector;
import frc.robot.util.TunableNumber;

public class Shoot extends CommandBase {
  private static final boolean enableRumble = true;
  private static final TunableNumber rumblePercent =
      new TunableNumber("Shoot/RumblePercent");
  private static final TunableNumber rumbleDurationSecs =
      new TunableNumber("Shoot/RumbleDuration");

  private static final TunableNumber towerSpeed =
      new TunableNumber("Shoot/TowerSpeed");
  private static final TunableNumber kickerSpeed =
      new TunableNumber("Shoot/KickerSpeed");

  private final Tower tower;
  private final Kicker kicker;
  private final Hood hood;
  private final LedSelector leds;

  private final Consumer<Double> rumbleConsumer;
  private boolean rumbleLastTripped = false;
  private boolean rumbleActive = false;
  private final Timer rumbleTimer = new Timer();

  /** Creates a new Shoot. Runs the tower and kicker to fire cargo. */
  public Shoot(Tower tower, Kicker kicker, Hood hood, LedSelector leds) {
    this(tower, kicker, hood, leds, x -> {
    });
  }

  /** Creates a new Shoot. Runs the tower and kicker to fire cargo. */
  public Shoot(Tower tower, Kicker kicker, Hood hood, LedSelector leds,
      Consumer<Double> rumbleConsumer) {
    addRequirements(tower, kicker, hood);
    this.tower = tower;
    this.kicker = kicker;
    this.hood = hood;
    this.leds = leds;
    this.rumbleConsumer = rumbleConsumer;

    rumblePercent.setDefault(0.5);
    rumbleDurationSecs.setDefault(0.2);

    towerSpeed.setDefault(0.6);
    kickerSpeed.setDefault(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.runPercent(towerSpeed.get());
    kicker.runPercent(kickerSpeed.get());
    hood.moveToShootPosition();
    leds.setShooting(true);

    rumbleLastTripped = tower.getUpperCargoSensor();
    rumbleActive = false;
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/Shoot", true);
    boolean rumbleTripped = tower.getUpperCargoSensor();
    if (rumbleLastTripped && !rumbleTripped && enableRumble) {
      rumbleActive = true;
      rumbleTimer.reset();
    }
    if (rumbleActive && !rumbleTimer.hasElapsed(rumbleDurationSecs.get())) {
      rumbleConsumer.accept(rumblePercent.get());
    } else {
      rumbleConsumer.accept(0.0);
    }
    rumbleLastTripped = rumbleTripped;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.stop();
    kicker.stop();
    leds.setShooting(false);
    rumbleTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
