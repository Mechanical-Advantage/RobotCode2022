// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.TunableNumber;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Shoot extends CommandBase {
  private static final boolean enableRumble = true;
  private static final TunableNumber rumblePercent = new TunableNumber("Shoot/RumblePercent");
  private static final TunableNumber rumbleDurationSecs = new TunableNumber("Shoot/RumbleDuration");

  private final Feeder feeder;
  private final Leds leds;

  private final Consumer<Double> rumbleConsumer;
  private boolean rumbleLastTripped = false;
  private boolean rumbleActive = false;
  private final Timer rumbleTimer = new Timer();

  /** Creates a new Shoot. Runs the feeder to fire cargo. */
  public Shoot(Feeder feeder, Leds leds) {
    this(feeder, leds, x -> {});
  }

  /** Creates a new Shoot. Runs the feeder to fire cargo. */
  public Shoot(Feeder feeder, Leds leds, Consumer<Double> rumbleConsumer) {
    this.feeder = feeder;
    this.leds = leds;
    this.rumbleConsumer = rumbleConsumer;

    rumblePercent.setDefault(0.5);
    rumbleDurationSecs.setDefault(0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.requestShoot(true);
    leds.setShooting(true);

    rumbleLastTripped = feeder.getUpperProxSensor();
    rumbleActive = false;
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.getInstance().recordOutput("ActiveCommands/Shoot", true);

    boolean rumbleTripped = feeder.getUpperProxSensor();
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
    feeder.requestShoot(false);
    leds.setShooting(false);
    rumbleConsumer.accept(0.0);
    rumbleTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
