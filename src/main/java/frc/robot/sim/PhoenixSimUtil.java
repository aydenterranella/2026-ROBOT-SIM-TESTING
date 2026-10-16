// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Sim-state bridge adapters between MapleSim and Phoenix 6.
 *
 * <p>Each adapter implements {@link SimulatedMotorController}. On every MapleSim
 * physics sub-tick (~5 per 20ms), MapleSim calls {@link #updateControlSignal} with
 * the simulated encoder + mechanism state. The adapter pushes that state into the
 * Phoenix sim state so CTRE's odometry reads correct positions/velocities, and
 * returns the voltage CTRE wants applied so MapleSim's motor physics can advance.
 *
 * <p>Pattern adapted from the Shenzhen-Robotics-Alliance AKit-MapleSim-Enhanced
 * reference.
 */
public final class PhoenixSimUtil {
  private PhoenixSimUtil() {}

  /**
   * Strips the real-robot calibration the MapleSim ↔ Phoenix 6 bridge can't represent.
   *
   * <p>The simulated wheels start at a known zero pose, and the adapters below push MapleSim's
   * raw rotor and CANcoder positions with no encoder offset and no motor or encoder inversion.
   * Keeping the robot's CANcoder offsets and inversion flags makes the steer PID chase a phantom
   * error, which shows up as the chassis sitting in place and vibrating.
   *
   * <p>The coupling ratio is zeroed too. On the robot, steering turns the drive wheel through the
   * module gears (4.5 drive rotor turns per azimuth turn) and Phoenix backs that out of odometry
   * and adds it to drive requests. MapleSim has no such coupling, so in sim the compensation read
   * every steer move as wheel speed (~0.5 m/s for an X-lock) and drove the wheels to match. The
   * stationary X-lock then saw the robot "moving", let go, re-aimed and locked again every
   * ~0.15 s, spinning the wheels while the robot sat still. With no coupling on either side, sim
   * nets out the same as the robot.
   *
   * <p>Everything else runs exactly as on the robot: steer gains, Voltage closed loop, 12.1:1,
   * FusedCANcoder, drive gains. Headless step tests showed the sim-only steer overrides this
   * used to apply (kP 70, kD 4.5, kV 1.91, 16:1) only made up for a too-heavy MapleSim steer
   * inertia (see {@code Swerve.initMapleSim}); the 16:1 ratio had no measurable effect, and the
   * friction/inertia overrides only feed CTRE's SimSwerveDrivetrain, which this project never
   * steps.
   *
   * <p>No-op on the real robot. Mutates and returns the same instance.
   */
  public static SwerveModuleConstants<?, ?, ?> regulateForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    if (RobotBase.isReal()) return moduleConstants;
    return moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withCouplingGearRatio(0);
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState simState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.simState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      simState.setRawRotorPosition(encoderAngle);
      simState.setRotorVelocity(encoderVelocity);
      simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return simState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXWithRemoteCANcoderSim extends TalonFXMotorControllerSim {
    private final CANcoderSimState cancoderSimState;

    public TalonFXWithRemoteCANcoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.cancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      cancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      cancoderSimState.setRawPosition(mechanismAngle);
      cancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
