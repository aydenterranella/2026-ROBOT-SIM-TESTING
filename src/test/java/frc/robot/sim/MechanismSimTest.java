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
import frc.lib.team9410.subsystems.PositionSubsystem;
import frc.lib.team9410.subsystems.VelocitySubsystem;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
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
    PositionSubsystem wrist = new PositionSubsystem(IntakeConstants.WRIST_CONFIG);
    ArmMechanismSim sim =
        new ArmMechanismSim(
            new ArmMechanismSim.Config(
                "IntakeWrist",
                IntakeConstants.WRIST_CONFIG,
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

    // Note: PositionSubsystem's constructor immediately commands the default (idle) position,
    // so the arm is already moving here. We only check that the TalonFX's fused reading agrees
    // with the simulated arm.
    run(0.5, () -> sim.update(LOOP_PERIOD_SECS));
    assertEquals(sim.getPositionRotations(), wrist.getPositionRotations(), 0.01,
        "TalonFX should report the same position the sim wrote into the sensors");

    // Deploy.
    wrist.setPositionRotations(IntakeConstants.INTAKE_MAX);
    run(3.0, () -> sim.update(LOOP_PERIOD_SECS));
    assertEquals(IntakeConstants.INTAKE_MAX, sim.getPositionRotations(), 0.02, "wrist should reach down");
    assertEquals(sim.getPositionRotations(), wrist.getPositionRotations(), 0.01,
        "sim and TalonFX should agree");
    assertEquals(Math.toDegrees(INTAKE_REF_ANGLE_RAD), Math.toDegrees(sim.getAngleRads()), 8.0,
        "deployed arm should be near horizontal, like the CAD");

    // Back to idle.
    wrist.setPositionRotations(IntakeConstants.INTAKE_IDLE);
    run(2.0, () -> sim.update(LOOP_PERIOD_SECS));
    assertEquals(IntakeConstants.INTAKE_IDLE, sim.getPositionRotations(), 0.02, "wrist should reach idle");
  }

  @Test
  void hoodTracksSetpoint() throws InterruptedException {
    PositionSubsystem hoodSubsystem = new PositionSubsystem(ShooterConstants.HOOD_CONFIG);
    ArmMechanismSim sim =
        new ArmMechanismSim(
            new ArmMechanismSim.Config(
                "Hood",
                ShooterConstants.HOOD_CONFIG,
                HOOD_GEARBOX,
                HOOD_J_KG_M2,
                HOOD_ARM_LENGTH_M,
                HOOD_REF_POSITION_ROT,
                HOOD_REF_ANGLE_RAD,
                HOOD_ARM_RAD_PER_MECH_RAD,
                HOOD_HARD_STOP_DOWN_ROT,
                HOOD_HARD_STOP_UP_ROT,
                HOOD_START_ROT),
            hoodSubsystem.getPositionMotor());

    run(0.5, () -> sim.update(LOOP_PERIOD_SECS));

    double target = 0.10;
    hoodSubsystem.setPositionRotations(target);
    run(3.0, () -> sim.update(LOOP_PERIOD_SECS));
    // Looser tolerance: kG is 0 in code, so gravity leaves a small steady-state droop.
    assertEquals(target, sim.getPositionRotations(), 0.02, "hood should reach the setpoint");
    assertEquals(sim.getPositionRotations(), hoodSubsystem.getPositionRotations(), 0.01,
        "sim and TalonFX should agree");
  }

  @Test
  void flywheelSpinsUp() throws InterruptedException {
    VelocitySubsystem shooter = new VelocitySubsystem(ShooterConstants.FLYWHEEL_CONFIG);
    RollerMechanismSim sim =
        new RollerMechanismSim(
            "Flywheel",
            shooter.getVelocityMotor(),
            FLYWHEEL_GEARBOX,
            FLYWHEEL_J_KG_M2,
            FLYWHEEL_FRICTION_VOLTS);

    double target = 75.0; // rps, middle of the shot LUT
    shooter.setVelocity(target);
    run(2.0, () -> sim.update(LOOP_PERIOD_SECS));

    assertEquals(target, sim.getVelocityRps(), 2.0, "flywheel should reach the setpoint");
    assertEquals(sim.getVelocityRps(),
        shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble(), 1.0,
        "TalonFX should report the simulated speed");
  }
}
