// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableWaitCommand;

public class AutoClimb extends SequentialCommandGroup {
  private static final double downPullVolts = 6.0;

  private static final TunableNumber midToHighStartDelay =
      new TunableNumber("AutoClimb/MidToHighStartDelay", 0.1);
  private static final TunableNumber midToHighPullDelay =
      new TunableNumber("AutoClimb/MidToHighPullDelay", 0.6);
  private static final TunableNumber highToTraversalStartDelay =
      new TunableNumber("AutoClimb/HighToTraversalStartDelay", 0.6);
  private static final TunableNumber highToTraversalPullDelay =
      new TunableNumber("AutoClimb/HighToTraversalPullDelay", 0.5);

  /** Creates a new AutoClimb. */
  public AutoClimb(Climber climber, Drive drive, Leds leds) {
    addCommands(
        new RunClimberToPosition(climber, climber.minPositionRad.get()),
        new RunClimberToBottom(climber, downPullVolts),
        new TunableWaitCommand(midToHighStartDelay),
        new RunClimberToPosition(climber, climber.maxPositionRad.get()),
        new TunableWaitCommand(midToHighPullDelay),
        new RunClimberToPosition(climber, climber.minPositionRad.get()),
        new RunClimberToBottom(climber, downPullVolts),
        new TunableWaitCommand(highToTraversalStartDelay),
        new RunClimberToPosition(climber, climber.maxPositionRad.get()),
        new TunableWaitCommand(highToTraversalPullDelay),
        new RunClimberToPosition(climber, 10.0),
        new InstantCommand(() -> leds.setClimbSuccess(true)));
  }
}
