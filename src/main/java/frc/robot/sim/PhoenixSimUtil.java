// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
   * Strips real-robot calibration that breaks the MapleSim ↔ Phoenix 6 sim loop.
   *
   * <p>The simulated wheels start at a known zero pose with no encoder offset and no motor
   * inversion. Real-robot CANcoder offsets and motor/encoder inversion flags cause the steer
   * PID to chase a phantom error in sim, which manifests as the chassis sitting in place and
   * vibrating. Calling this on each module's constants before construction fixes that.
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
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSteerMotorGearRatio(16.0)
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.05))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
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
