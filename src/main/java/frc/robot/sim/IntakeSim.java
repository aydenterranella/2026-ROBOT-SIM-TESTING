// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static frc.robot.sim.MechanismSimConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Sim intake: picks fuel up off the field and hands it to {@link FuelPathSim}, which carries it
 * through the robot.
 *
 * <p>MapleSim's own intake swallows every ball the instant it touches a zone at the bumper. This one
 * instead only takes fuel sitting under the deployed rollers (out past the bumper), only while the
 * {@code intakeActive} supplier says the arm is down and the rollers are spinning, and no faster
 * than {@link MechanismSimConstants#INTAKE_BALLS_PER_SECOND}. Fuel it can't take yet stays on the
 * field and gets shoved by the bumper, like a jam.
 */
public class IntakeSim {
  private final SwerveDriveSimulation driveSim;
  private final FuelPathSim fuelPath;
  private final BooleanSupplier intakeActive;

  /** Pickups banked toward the next ball (rate limit). */
  private double pickupAllowance = 0.0;

  public IntakeSim(SwerveDriveSimulation driveSim, FuelPathSim fuelPath, BooleanSupplier intakeActive) {
    this.driveSim = driveSim;
    this.fuelPath = fuelPath;
    this.intakeActive = intakeActive;
  }

  /** Call once per {@code simulationPeriodic()}, before the arena steps. */
  public void update() {
    if (!intakeActive.getAsBoolean()) {
      pickupAllowance = 0.0;
      return;
    }
    // Bank at most one ball's worth so it can't save up a burst while nothing is in reach.
    pickupAllowance = Math.min(1.0, pickupAllowance + LOOP_PERIOD_SECS * INTAKE_BALLS_PER_SECOND);

    Pose2d robot = driveSim.getSimulatedDriveTrainPose();
    for (GamePieceOnFieldSimulation piece : SimulatedArena.getInstance().gamePiecesOnField()) {
      if (pickupAllowance < 1.0 || !fuelPath.hasRoom()) break;
      if (!"Fuel".equals(piece.getType())) continue;
      Translation2d rel = piece.getPoseOnField().getTranslation().minus(robot.getTranslation())
          .rotateBy(robot.getRotation().unaryMinus());
      boolean underRollers = rel.getX() >= INTAKE_ZONE_MIN_X_M && rel.getX() <= INTAKE_ZONE_MAX_X_M
          && Math.abs(rel.getY()) <= INTAKE_ZONE_HALF_WIDTH_M;
      if (!underRollers) continue;

      SimulatedArena.getInstance().removeGamePiece(piece);
      fuelPath.addFromIntake(new Translation3d(rel.getX(), rel.getY(), FUEL_RADIUS_M));
      pickupAllowance -= 1.0;
    }
  }
}
