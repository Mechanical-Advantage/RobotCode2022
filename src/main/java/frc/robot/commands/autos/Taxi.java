// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableNumber;

public class Taxi extends CommandBase {
  private static TunableNumber longDuration =
      new TunableNumber("SimpleTaxi/LongDurationSecs");
  private static TunableNumber shortDuration =
      new TunableNumber("SimpleTaxi/ShortDurationSecs");
  private static TunableNumber speed = new TunableNumber("SimpleTaxi/Speed");

  private final Drive drive;
  private final boolean isLong;
  private final Timer timer = new Timer();

  /** Creates a new Taxi. Drives straight forwards to reliably taxi. */
  public Taxi(Drive drive, boolean isLong) {
    addRequirements(drive);
    this.drive = drive;
    this.isLong = isLong;

    longDuration.setDefault(1.0);
    shortDuration.setDefault(0.7);
    speed.setDefault(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drivePercent(speed.get(), speed.get());
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(isLong ? longDuration.get() : shortDuration.get());
  }
}
