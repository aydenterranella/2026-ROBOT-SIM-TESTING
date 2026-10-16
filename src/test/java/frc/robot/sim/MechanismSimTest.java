// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static frc.robot.sim.MechanismSimConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.powerlib.subsystems.AbsolutePositionSubsystem;
import frc.powerlib.subsystems.PowerSubsystem;
import frc.powerlib.subsystems.VelocitySubsystem;
import frc.robot.constants.IntakeRollerConstants;
import frc.robot.constants.IntakeWristConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterHoodConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Closed-loop checks for the mechanism sims: build the REAL subsystems (same configs, same
 * Phoenix control loops), attach the sims, command a setpoint, and make sure the simulated
 * mechanism actually gets there.
 *
 * <p>The main thing this catches is sign mistakes. If any of the CANcoder direction / offset /
 * ratio conversions in {@link ArmMechanismSim} were flipped, the Phoenix PID would push the arm
 * the wrong way and it would slam into a hard stop instead of settling.
 *
 * <p>Phoenix's simulated devices run in real time on their own thread, so each step sleeps for
 * one loop period. These tests take a few seconds each.
 */
class MechanismSimTest {

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
    // Same switch Robot() flips: real TalonFX/CANcoder devices instead of the kinematic IOSim.
    PowerSubsystem.usePhysicsSimulation(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /** Runs {@code seconds} of simulated time, stepping the given sims each loop. */
  private static void run(double seconds, Runnable step) throws InterruptedException {
    int loops = (int) Math.round(seconds / LOOP_PERIOD_SECS);
    for (int i = 0; i < loops; i++) {
      Unmanaged.feedEnable(100); // keep Phoenix devices enabled
      DriverStationSim.notifyNewData();
      step.run();
      Thread.sleep((long) (LOOP_PERIOD_SECS * 1000));
    }
  }

  @Test
  void intakeWristDeploysAndStows() throws InterruptedException {
    AbsolutePositionSubsystem wrist =
        new AbsolutePositionSubsystem(IntakeWristConstants.INTAKE_WRIST_CONFIG);
    assertTrue(wrist.isConfigured(), "wrist devices should configure in physics sim");
    ArmMechanismSim sim =
        new ArmMechanismSim(
            new ArmMechanismSim.Config(
                "IntakeWrist",
                IntakeWristConstants.INTAKE_WRIST_CONFIG,
                INTAKE_WRIST_GEARBOX,
                INTAKE_WRIST_J_KG_M2,
                INTAKE_ARM_LENGTH_M,
                INTAKE_REF_POSITION_ROT,
                INTAKE_REF_ANGLE_RAD,
                INTAKE_ARM_RAD_PER_MECH_RAD,
                INTAKE_HARD_STOP_DOWN_ROT,
                INTAKE_HARD_STOP_UP_ROT,
                INTAKE_START_ROT),
            wrist.getPositionMotor());
    Runnable step = () -> {
      sim.update(LOOP_PERIOD_SECS);
      wrist.periodic(); // PowerLib reads feedback into inputs here
    };

    // Note: AbsolutePositionSubsystem's constructor immediately commands the default (idle)
    // position, so the arm is already moving here. We only check that the TalonFX's fused reading
    // agrees with the simulated arm.
    run(0.5, step);
    wrist.periodic(); // inputs were read before the last sleep; refresh them
    assertEquals(sim.getPositionRotations(), wrist.getPositionRotations(), 0.01,
        "TalonFX should report the same position the sim wrote into the sensors");

    // Deploy.
    wrist.setPositionRotations(IntakeWristConstants.INTAKE_MAX);
    run(3.0, step);
    wrist.periodic();
    assertEquals(IntakeWristConstants.INTAKE_MAX, sim.getPositionRotations(), 0.02, "wrist should reach down");
    assertEquals(sim.getPositionRotations(), wrist.getPositionRotations(), 0.01,
        "sim and TalonFX should agree");
    assertEquals(Math.toDegrees(INTAKE_REF_ANGLE_RAD), Math.toDegrees(sim.getAngleRads()), 8.0,
        "deployed arm should be near horizontal, like the CAD");

    // Back to idle.
    wrist.setPositionRotations(IntakeWristConstants.INTAKE_IDLE);
    run(2.0, step);
    assertEquals(IntakeWristConstants.INTAKE_IDLE, sim.getPositionRotations(), 0.02, "wrist should reach idle");
  }

  @Test
  void hoodTracksSetpoint() throws InterruptedException {
    AbsolutePositionSubsystem hoodSubsystem =
        new AbsolutePositionSubsystem(ShooterHoodConstants.SHOOTER_HOOD_CONFIG);
    ArmMechanismSim sim =
        new ArmMechanismSim(
            new ArmMechanismSim.Config(
                "Hood",
                ShooterHoodConstants.SHOOTER_HOOD_CONFIG,
                HOOD_GEARBOX,
                HOOD_J_KG_M2,
                HOOD_ARM_LENGTH_M,
                HOOD_REF_POSITION_ROT,
                HOOD_REF_ANGLE_RAD,
                HOOD_ARM_RAD_PER_MECH_RAD,
                HOOD_HARD_STOP_DOWN_ROT,
                HOOD_HARD_STOP_UP_ROT,
                HOOD_START_ROT,
                HOOD_ROTOR_PER_ARM_ROT),
            hoodSubsystem.getPositionMotor());
    Runnable step = () -> {
      sim.update(LOOP_PERIOD_SECS);
      hoodSubsystem.periodic();
    };

    run(0.5, step);

    double target = 0.07; // middle of the shot table's hood range
    hoodSubsystem.setPositionRotations(target);
    run(3.0, step);
    hoodSubsystem.periodic();
    // Looser tolerance: kG is 0 in code, so gravity leaves a small steady-state droop.
    assertEquals(target, sim.getPositionRotations(), 0.02, "hood should reach the setpoint");
    assertEquals(sim.getPositionRotations(), hoodSubsystem.getPositionRotations(), 0.01,
        "sim and TalonFX should agree");
  }

  @Test
  void flywheelSpinsUp() throws InterruptedException {
    VelocitySubsystem shooter = new VelocitySubsystem(ShooterConstants.SHOOTER_CONFIG);
    RollerMechanismSim sim =
        new RollerMechanismSim(
            "Flywheel",
            shooter.getVelocityMotor(),
            FLYWHEEL_GEARBOX,
            FLYWHEEL_J_KG_M2,
            FLYWHEEL_FRICTION_VOLTS);

    double target = 33.0; // rps, middle of the shot table
    shooter.setVelocity(target);
    run(2.0, () -> {
      sim.update(LOOP_PERIOD_SECS);
      shooter.periodic();
    });

    assertEquals(target, sim.getVelocityRps(), 2.0, "flywheel should reach the setpoint");
    assertEquals(sim.getVelocityRps(), shooter.inputs.velocityRotationsPerSecond, 1.0,
        "TalonFX should report the simulated speed");
  }

  @Test
  void stoppedRollerComesToRestWithoutChattering() throws InterruptedException {
    // The light intake roller used to flip between ±0.9 rps every loop once stopped (friction
    // carried it through zero each step), so Power Tool's tile kept changing sign.
    VelocitySubsystem roller = new VelocitySubsystem(IntakeRollerConstants.INTAKE_ROLLER_CONFIG);
    RollerMechanismSim sim =
        new RollerMechanismSim(
            "IntakeRoller",
            roller.getVelocityMotor(),
            INTAKE_ROLLER_GEARBOX,
            INTAKE_ROLLER_J_KG_M2,
            INTAKE_ROLLER_FRICTION_VOLTS);
    Runnable step = () -> {
      sim.update(LOOP_PERIOD_SECS);
      roller.periodic();
    };
    roller.setVelocity(INTAKE_ROLLER_INTAKING_RPS);
    run(1.0, step);
    roller.stopVelocity();
    roller.brake();
    run(0.5, step);

    int signChanges = 0;
    double last = sim.getVelocityRps();
    for (int i = 0; i < 25; i++) {
      run(LOOP_PERIOD_SECS, step);
      double now = sim.getVelocityRps();
      if (Math.signum(now) != 0 && Math.signum(last) != 0 && Math.signum(now) != Math.signum(last)) signChanges++;
      last = now;
    }
    assertEquals(0, signChanges, "stopped roller should not reverse back and forth");
    assertEquals(0.0, sim.getVelocityRps(), 0.05, "stopped roller should be at rest");
  }
}
