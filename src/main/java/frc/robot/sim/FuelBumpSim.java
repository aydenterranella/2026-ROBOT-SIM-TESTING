package frc.robot.sim;

import static frc.robot.sim.MechanismSimConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Makes fuel on the field roll over the bumps instead of sliding through them.
 *
 * <p>MapleSim's field is flat: the bumps aren't colliders (so robots can cross them), and fuel just
 * passes through them at floor height. This adds the slope back for fuel: on a ramp face, gravity
 * pulls the ball back down the slope (a rolling ball feels 5/7 g sin θ), so a slow ball rolls back
 * off and a fast one makes it over, and its drawn height follows the bump surface. Same bump
 * geometry as {@link RobotBumpSim}.
 */
public final class FuelBumpSim {
  private FuelBumpSim() {}

  private static final double GRAVITY = 9.81;

  /** Call once per {@code simulationPeriodic()}, before the arena steps. */
  public static void update() {
    for (GamePieceOnFieldSimulation piece : SimulatedArena.getInstance().gamePiecesOnField()) {
      Vector2 position = piece.getTransform().getTranslation();
      double slope = slopeAt(position.x, position.y);
      if (slope == 0.0) continue;
      // Downhill acceleration along field X for a ball rolling on a face rising at `slope` (dz/dx).
      double sinCos = slope / (1.0 + slope * slope); // sin θ cos θ for tan θ = slope
      double ax = -(5.0 / 7.0) * GRAVITY * sinCos;
      Vector2 v = piece.getLinearVelocity();
      piece.setLinearVelocity(v.x + ax * LOOP_PERIOD_SECS, v.y);
    }
  }

  /** Raises a fuel pose to sit on the bump surface wherever it is (no change on flat floor). */
  public static Pose3d onSurface(Pose3d fuel) {
    double h = heightAt(fuel.getX(), fuel.getY());
    if (h == 0.0) return fuel;
    return new Pose3d(fuel.getTranslation().plus(new Translation3d(0, 0, h)), fuel.getRotation());
  }

  /** Bump surface height (m) at a field point, or 0 off the bumps. */
  public static double heightAt(double x, double y) {
    for (int i = 0; i < RobotBumpSim.BUMP_LINE_STARTS.length; i++) {
      Translation3d s = RobotBumpSim.BUMP_LINE_STARTS[i], e = RobotBumpSim.BUMP_LINE_ENDS[i];
      if (x < s.getX() || x > e.getX() || y < s.getY() || y > e.getY()) continue;
      return s.getZ() + (e.getZ() - s.getZ()) * (x - s.getX()) / (e.getX() - s.getX());
    }
    return 0.0;
  }

  /** Bump surface slope dz/dx at a field point, or 0 off the bumps. */
  public static double slopeAt(double x, double y) {
    for (int i = 0; i < RobotBumpSim.BUMP_LINE_STARTS.length; i++) {
      Translation3d s = RobotBumpSim.BUMP_LINE_STARTS[i], e = RobotBumpSim.BUMP_LINE_ENDS[i];
      if (x < s.getX() || x > e.getX() || y < s.getY() || y > e.getY()) continue;
      return (e.getZ() - s.getZ()) / (e.getX() - s.getX());
    }
    return 0.0;
  }
}
