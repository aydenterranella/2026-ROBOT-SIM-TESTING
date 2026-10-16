// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Physics sim for anything that just spins: flywheel, feeder, spindexer, intake roller.
 *
 * <h2>How it works (once per robot loop)</h2>
 * <ol>
 *   <li>Ask the simulated TalonFX what voltage its closed-loop controller is applying.
 *   <li>Feed that voltage into a WPILib {@link DCMotorSim} (motor curve + inertia).
 *   <li>Write the resulting rotor position/velocity back into the TalonFX sim state, so
 *       {@code getRotorVelocity()} etc. return realistic values on the next loop.
 * </ol>
 *
 * <p>Only the LEADER motor is simulated. Followers are folded into the {@link DCMotor} gearbox
 * (e.g. {@code getKrakenX60Foc(3)} for three flywheel motors), since they all push on the same
 * shaft.
 *
 * <p>All of these mechanisms are configured 1:1 in code (SensorToMechanismRatio = 1), so the
 * sim runs directly in rotor units and the inertia is the inertia reflected to the motor shaft.
 */
public class RollerMechanismSim {
  private final String name;
  private final TalonFXSimState talonSim;
  private final DCMotorSim physics;
  private final double frictionVolts;

  /**
   * @param name label used for dashboard keys
   * @param leader the leader TalonFX from the subsystem (followers are modeled via the gearbox)
   * @param gearbox combined motor model for all motors on this mechanism
   * @param jKgMetersSquared moment of inertia seen by the motor shaft
   * @param frictionVolts constant friction, expressed as the voltage it eats. Pass the SysId kS:
   *     kS is literally "volts needed to overcome friction", and without it the sim spins
   *     faster than the real mechanism (the controller's kS has nothing to cancel).
   */
  public RollerMechanismSim(
      String name, TalonFX leader, DCMotor gearbox, double jKgMetersSquared, double frictionVolts) {
    this.name = name;
    this.frictionVolts = frictionVolts;
    this.talonSim = leader.getSimState();
    // Gearing = 1.0: the code works in rotor rotations, so we do too.
    this.physics =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, jKgMetersSquared, 1.0), gearbox);

    // Let MapleSim's battery model see our current draw, so spinning up the flywheel
    // sags the simulated battery voltage like it would on the real robot.
    SimulatedBattery.addElectricalAppliances(() -> Amps.of(Math.abs(talonSim.getSupplyCurrent())));
  }

  /** Steps the physics by {@code dtSeconds} and pushes the new state into the TalonFX. */
  public void update(double dtSeconds) {
    talonSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

    // getMotorVoltage() is in the sim state's "chassis" frame. Phoenix applies the motor's
    // Inverted setting internally, so we never have to flip signs here.
    double appliedVolts = talonSim.getMotorVoltage();

    // Coulomb friction: always opposes motion. When stopped, it cancels up to kS volts so a
    // tiny command doesn't start the roller (just like the real one).
    double velocity = physics.getAngularVelocityRadPerSec();
    double friction =
        Math.abs(velocity) > 1e-3
            ? Math.copySign(frictionVolts, velocity)
            : MathUtil.clamp(appliedVolts, -frictionVolts, frictionVolts);

    physics.setInputVoltage(appliedVolts - friction);
    physics.update(dtSeconds);
    // Friction can stop the roller but never reverse it. Applied for a whole loop, it used to
    // carry a coasting roller through zero and back every step (the intake roller sat flipping
    // between ±0.9 rps once stopped). If it crossed zero without enough voltage to beat kS, it
    // stuck at zero instead.
    double after = physics.getAngularVelocityRadPerSec();
    if (Math.abs(velocity) > 1e-3 && Math.signum(after) != Math.signum(velocity)
        && Math.abs(appliedVolts) <= frictionVolts) {
      physics.setState(physics.getAngularPositionRad(), 0.0);
    }

    talonSim.setRawRotorPosition(physics.getAngularPositionRotations());
    talonSim.setRotorVelocity(
        Units.radiansToRotations(physics.getAngularVelocityRadPerSec()));
  }

  /** Simulated rotor speed in rotations per second (same units as the code's setpoints). */
  public double getVelocityRps() {
    return Units.radiansToRotations(physics.getAngularVelocityRadPerSec());
  }

  /** Total current through all motors on this mechanism (amps). */
  public double getCurrentDrawAmps() {
    return physics.getCurrentDrawAmps();
  }

  public String getName() {
    return name;
  }
}
