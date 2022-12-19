// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

/** Hood subsystem hardware interface for WPILib arm sim. */
public class HoodIOSim implements HoodIO {
  private static final double lengthMeters = Units.inchesToMeters(12.0);
  private static final double massKg = 2.5;
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          20.0,
          SingleJointedArmSim.estimateMOI(lengthMeters, massKg),
          lengthMeters,
          Units.degreesToRadians(10.0),
          Units.degreesToRadians(80.0),
          massKg,
          false);

  private double appliedVolts = 0.0;
  private final MechanismLigament2d hood;

  public HoodIOSim() {
    Mechanism2d mech = new Mechanism2d(2.0, 3.0);
    MechanismRoot2d robot = mech.getRoot("Robot", 1.0, 0.0);
    MechanismLigament2d tower =
        robot.append(new MechanismLigament2d("Tower", 1.0, 90.0, 20, new Color8Bit(Color.kGray)));
    hood =
        tower.append(new MechanismLigament2d("Hood", 0.5, 45.0, 10, new Color8Bit(Color.kSkyBlue)));
    hood.append(new MechanismLigament2d("Camera", 0.2, 90.0, 10, new Color8Bit(Color.kSkyBlue)));
    SmartDashboard.putData("Hood", mech);
  }

  public void updateInputs(HoodIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {};

    hood.setAngle(Units.radiansToDegrees(sim.getAngleRads()) - 90.0);
  }

  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }
}
