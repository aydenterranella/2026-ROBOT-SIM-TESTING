// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team9410.configs.PositionSubsystemConfig;
import frc.robot.Constants;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Physics sim for a pivoting arm driven by a TalonFX with a fused CANcoder — the setup
 * {@link frc.lib.team9410.subsystems.PositionSubsystem} uses for the intake wrist and the hood.
 *
 * <h2>The three "units" in play</h2>
 * It helps to keep these straight when reading the code:
 * <ul>
 *   <li><b>Physical angle θ</b> (radians) — what gravity cares about and what we draw. 0 = arm
 *       pointing straight forward (+X), positive = rotating up. The WPILib
 *       {@link SingleJointedArmSim} works in this unit.
 *   <li><b>Mechanism position P</b> (rotations) — what the code commands, e.g.
 *       {@code INTAKE_MAX = -0.452}. This is the TalonFX {@code getPosition()} value.
 *   <li><b>Raw sensor values</b> — what we have to write into the CANcoder and TalonFX sim states.
 *       These include the CANcoder magnet offset, sensor direction, and gear ratios.
 * </ul>
 *
 * <h2>How the TalonFX turns sensors into P (Phoenix 6, FusedCANcoder mode)</h2>
 * <pre>
 *   cancoderPos = direction * rawCancoder + magnetOffset     (what the CANcoder reports)
 *   P           = cancoderPos / SensorToMechanismRatio        (what the TalonFX reports)
 *   rotor       = P * |SensorToMechanismRatio| * RotorToSensorRatio
 * </pre>
 * The rotor line is the non-obvious one. Our configs use a NEGATIVE SensorToMechanismRatio (-1)
 * to flip the CANcoder, but Phoenix's closed loop still assumes positive motor output makes P go
 * UP. So the rotor has to move with P, not with the raw CANcoder. This was verified by
 * {@code MechanismSimTest}: with the rotor tied to the CANcoder instead, both arms drove straight
 * into their hard stops.
 * Each loop we go the other way: physics gives θ → we compute P → we compute the raw values that
 * would make the TalonFX report exactly that P. Because of that, the sim always agrees with
 * whatever offsets/ratios are in the constants files, even if they change.
 *
 * <h2>Mapping between θ and P</h2>
 * Linear, anchored at one known pose from the CAD:
 * {@code θ = refAngle + armRadPerMechRad * 2π * (P - refPosition)}.
 */
public class ArmMechanismSim {

  /**
   * Everything that describes one arm. Values come from {@link MechanismSimConstants}.
   *
   * @param name label used for dashboard keys
   * @param subsystemConfig the same config the real subsystem was built with (ratios, offsets)
   * @param gearbox combined motor model (leader + followers)
   * @param jKgMetersSquared arm moment of inertia about the pivot
   * @param armLengthMeters pivot to end of arm (used for gravity)
   * @param refPositionRot a code position whose physical angle we know from CAD...
   * @param refAngleRad ...and that physical angle
   * @param armRadPerMechRad physical radians per mechanism radian (1.0 if the CANcoder is on the
   *     pivot; see the hood note in MechanismSimConstants)
   * @param hardStopLowRot lowest reachable code position (hard stop)
   * @param hardStopHighRot highest reachable code position (hard stop)
   * @param startPositionRot where the arm starts when the sim boots
   */
  public record Config(
      String name,
      PositionSubsystemConfig subsystemConfig,
      DCMotor gearbox,
      double jKgMetersSquared,
      double armLengthMeters,
      double refPositionRot,
      double refAngleRad,
      double armRadPerMechRad,
      double hardStopLowRot,
      double hardStopHighRot,
      double startPositionRot) {}

  /**
   * PositionSubsystem hard-codes the CANcoder to Clockwise_Positive, which flips the sign of the
   * raw reading. Keep this in sync if that library code ever changes.
   */
  private static final double CANCODER_DIRECTION_SIGN = -1.0;

  private final Config config;
  private final TalonFXSimState talonSim;
  private final CANcoderSimState cancoderSim;
  private final SingleJointedArmSim physics;

  private final double sensorToMechanism;
  private final double rotorToSensor;
  private final double magnetOffsetRot;
  /** +1 if the leader motor is not inverted, -1 if it is (see {@link #readInvertSign}). */
  private final double talonInvertSign;
  /** Converts TalonFX output voltage into "volts pushing θ positive". */
  private final double voltageToPhysicalSign;

  /**
   * @param config arm description
   * @param leader the subsystem's position motor ({@code PositionSubsystem.getPositionMotor()})
   */
  public ArmMechanismSim(Config config, TalonFX leader) {
    this.config = config;
    this.talonSim = leader.getSimState();

    // The subsystem builds its CANcoder privately. Phoenix 6 lets a second object talk to the
    // same device ID, so we make our own handle just to reach its sim state.
    var cancoderConfig = config.subsystemConfig().cancoderConfig();
    @SuppressWarnings("resource") // lives for the whole sim session
    CANcoder cancoder = new CANcoder(cancoderConfig.encoderId(), Constants.CanBus.CANIVORE_BUS);
    this.cancoderSim = cancoder.getSimState();

    var lead = config.subsystemConfig().leadConfig();
    this.sensorToMechanism = lead.sensorToMechanismRatio();
    this.rotorToSensor = lead.rotorToSensorRatio();
    this.magnetOffsetRot = cancoderConfig.magnetOffsetRotations();
    this.talonInvertSign = readInvertSign(leader);

    // Positive output (in the motor's own frame) makes P go up, and θ grows with
    // armRadPerMechRad * P. See the class docs for why S2M's sign doesn't appear here.
    this.voltageToPhysicalSign =
        talonInvertSign * Math.signum(rotorToSensor) * Math.signum(config.armRadPerMechRad());

    // Motor rotations per ARM rotation. Example (intake): 44.44 rotor turns per CANcoder turn,
    // CANcoder 1:1 with the arm → 44.44.
    double gearing =
        Math.abs(rotorToSensor * sensorToMechanism) / Math.abs(config.armRadPerMechRad());

    double angleA = angleFromPosition(config.hardStopLowRot());
    double angleB = angleFromPosition(config.hardStopHighRot());
    this.physics =
        new SingleJointedArmSim(
            config.gearbox(),
            gearing,
            config.jKgMetersSquared(),
            config.armLengthMeters(),
            Math.min(angleA, angleB),
            Math.max(angleA, angleB),
            true, // simulate gravity
            angleFromPosition(config.startPositionRot()));

    SimulatedBattery.addElectricalAppliances(() -> Amps.of(Math.abs(talonSim.getSupplyCurrent())));

    // Write the starting pose into the sensors right away so the TalonFX boots at the right spot.
    pushSensorState();
  }

  /** Steps the physics by {@code dtSeconds} and pushes new sensor values into the sim states. */
  public void update(double dtSeconds) {
    var battery = SimulatedBattery.getBatteryVoltage();
    talonSim.setSupplyVoltage(battery);
    cancoderSim.setSupplyVoltage(battery);

    physics.setInputVoltage(talonSim.getMotorVoltage() * voltageToPhysicalSign);
    physics.update(dtSeconds);

    pushSensorState();
  }

  /** Converts the physics state into raw CANcoder + rotor values (see class docs for the math). */
  private void pushSensorState() {
    double mechPos = positionFromAngle(physics.getAngleRads());
    double mechVel =
        physics.getVelocityRadPerSec() / (2.0 * Math.PI * config.armRadPerMechRad());

    // What the CANcoder should REPORT (after its offset/direction are applied)...
    double cancoderPos = mechPos * sensorToMechanism;
    double cancoderVel = mechVel * sensorToMechanism;
    // ...and the raw value that produces it.
    cancoderSim.setRawPosition((cancoderPos - magnetOffsetRot) * CANCODER_DIRECTION_SIGN);
    cancoderSim.setVelocity(cancoderVel * CANCODER_DIRECTION_SIGN);

    // The rotor moves WITH P (see class docs). Raw rotor values are in the un-inverted frame,
    // hence talonInvertSign.
    double rotorPerMech = Math.abs(sensorToMechanism) * rotorToSensor;
    talonSim.setRawRotorPosition(mechPos * rotorPerMech * talonInvertSign);
    talonSim.setRotorVelocity(mechVel * rotorPerMech * talonInvertSign);
  }

  /** Code position (rotations) → physical arm angle (radians). */
  public double angleFromPosition(double positionRot) {
    return config.refAngleRad()
        + config.armRadPerMechRad() * 2.0 * Math.PI * (positionRot - config.refPositionRot());
  }

  /** Physical arm angle (radians) → code position (rotations). */
  public double positionFromAngle(double angleRad) {
    return config.refPositionRot()
        + (angleRad - config.refAngleRad()) / (2.0 * Math.PI * config.armRadPerMechRad());
  }

  /** Simulated physical arm angle in radians (0 = forward, + = up). */
  public double getAngleRads() {
    return physics.getAngleRads();
  }

  /** Simulated arm position in code units (what {@code getPositionRotations()} should read). */
  public double getPositionRotations() {
    return positionFromAngle(physics.getAngleRads());
  }

  public double getCurrentDrawAmps() {
    return physics.getCurrentDrawAmps();
  }

  public String getName() {
    return config.name();
  }

  /**
   * Reads the motor's Inverted setting. The sim state always works in the un-inverted frame, so
   * we need this to translate rotor values.
   */
  private static double readInvertSign(TalonFX motor) {
    var output = new MotorOutputConfigs();
    motor.getConfigurator().refresh(output);
    return output.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0;
  }
}
