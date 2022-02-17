// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.TunableNumber;

public class PrepareShooter extends CommandBase {

  private static final TunableNumber lowerFenderBigRpm =
      new TunableNumber("PrepareShooter/LowerFenderBigRPM");
  private static final TunableNumber lowerFenderLitteRpm =
      new TunableNumber("PrepareShooter/LowerFenderLittleRPM");

  private static final TunableNumber upperFenderBigRpm =
      new TunableNumber("PrepareShooter/UpperFenderBigRPM");
  private static final TunableNumber upperFenderLittleRpm =
      new TunableNumber("PrepareShooter/UpperFenderLittleRPM");

  private static final PolynomialRegression upperTarmacBigRegression =
      new PolynomialRegression(new double[] {0.0}, new double[] {0.0}, 2, "x");
  private static final PolynomialRegression upperTarmacLittleRegression =
      new PolynomialRegression(new double[] {0.0}, new double[] {0.0}, 2, "x");

  private final Drive drive;
  private final Flywheels flywheels;
  private final Hood hood;
  private final ShooterPreset preset;

  /**
   * Creates a new PrepareShooter. Runs the flywheel and sets the hood position for the given
   * preset.
   */
  public PrepareShooter(Drive drive, Flywheels flywheels, Hood hood,
      ShooterPreset preset) {
    addRequirements(flywheels, hood);
    this.drive = drive;
    this.flywheels = flywheels;
    this.hood = hood;
    this.preset = preset;

    lowerFenderBigRpm.setDefault(0.0);
    lowerFenderLitteRpm.setDefault(0.0);

    upperFenderBigRpm.setDefault(0.0);
    upperFenderLittleRpm.setDefault(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean raised = false;
    double bigSpeed = 0.0, littleSpeed = 0.0;
    switch (preset) {
      case LOWER_FENDER:
        raised = true;
        bigSpeed = lowerFenderBigRpm.get();
        littleSpeed = lowerFenderLitteRpm.get();
        break;
      case UPPER_FENDER:
        raised = false;
        bigSpeed = upperFenderBigRpm.get();
        littleSpeed = upperFenderLittleRpm.get();
        break;
      case UPPER_TARMAC:
        raised = false;
        double distance = drive.getPose().getTranslation()
            .getDistance(FieldConstants.hubCenter);
        bigSpeed = upperTarmacBigRegression.predict(distance);
        littleSpeed = upperTarmacLittleRegression.predict(distance);
        break;
    }
    hood.setRaised(raised);
    flywheels.runVelocity(bigSpeed, littleSpeed);
    Logger.getInstance().recordOutput("ActiveCommands/PrepareShooter", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheels.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum ShooterPreset {
    LOWER_FENDER, UPPER_FENDER, UPPER_TARMAC
  }
}
