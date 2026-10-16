package frc.robot.sim;

import static frc.robot.sim.MechanismSimConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * Every fuel ball inside the robot, from the intake to the shooter:
 *
 * <ol>
 *   <li><b>Feeding in</b> — a ball the intake picks up arcs up over the roller and is dropped
 *       into the hopper.
 *   <li><b>Hopper</b> — loose balls under real physics (robot frame): gravity, bumping into each
 *       other and the hopper walls, piling up wherever they land. Balls touching the spinning
 *       floor get dragged around with it and drift out to its edge; balls that land up front roll
 *       down the ramp onto it.
 *   <li><b>Feeder</b> — a ball that reaches the feeder entrance (back edge of the spindexer) is
 *       pulled in when the feeder is running and has room, and carried up to the flywheel.
 *   <li><b>Shot</b> — a ball reaching the flywheel is handed back to the caller to launch.
 * </ol>
 *
 * <p>The spindexer and feeder move at their simulated speeds, so the fire rate and the delay before
 * the first shot come out of the mechanisms rather than a timer. The real robot only runs the
 * feeder and spindexer when it's ready to fire, so that alone gates the shots.
 *
 * <p>The hopper physics is position-based (each substep: move, then push overlapping balls apart
 * and back inside the walls, then take velocity from how far each ball actually moved). That's
 * cheap and stable for a pile of ~65 balls, and inelastic, which suits foam fuel.
 */
public class FuelPathSim {
  /** Balls between the intake and the hopper: where they were picked up, and when. */
  private record Incoming(Translation3d start, double startTime) {}

  /** A loose ball in the hopper. Robot frame, meters and m/s. */
  private static final class Ball {
    double x, y, z, vx, vy, vz, px, py, pz;
    boolean onFloor;

    Ball(double x, double y, double z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
  }

  private final List<Incoming> incoming = new ArrayList<>();
  private final List<Ball> hopper = new ArrayList<>();
  /** Distance (m) each feeder ball has traveled along the feeder path, oldest first. */
  private final List<Double> feeder = new ArrayList<>();

  private static final double R = FUEL_RADIUS_M;
  private static final double GRAVITY = 9.81;
  private static final int SUBSTEPS = 4;
  private static final int SOLVER_ITERATIONS = 3;
  private static final double FEEDER_SPACING_M = 2 * R + 0.005;
  private static final double FEEDER_PATH_LENGTH_M = pathLength();

  /** Total balls inside the robot, wherever they are. */
  public int getCount() {
    return incoming.size() + hopper.size() + feeder.size();
  }

  /** Room for one more? */
  public boolean hasRoom() {
    return getCount() < HOPPER_CAPACITY;
  }

  /** A ball the intake just picked up at {@code start} (robot-relative). */
  public void addFromIntake(Translation3d start) {
    incoming.add(new Incoming(start, Timer.getFPGATimestamp()));
  }

  /**
   * Empties the robot and puts {@code count} balls in the hopper (tests, resets). They start in a
   * loose stack and settle under gravity like anything else.
   */
  public void setCount(int count) {
    incoming.clear();
    hopper.clear();
    feeder.clear();
    int n = Math.max(0, Math.min(count, HOPPER_CAPACITY));
    for (int i = 0; i < n; i++) {
      Translation3d start = HOPPER_START_SPOTS[i % HOPPER_START_SPOTS.length];
      hopper.add(new Ball(start.getX(), start.getY(), start.getZ() + 0.2 * (i / HOPPER_START_SPOTS.length)));
    }
  }

  /**
   * Steps everything one loop.
   *
   * @param spindexerRps simulated spindexer rotor speed
   * @param feederRps simulated feeder rotor speed (negative = feeding toward the shooter)
   * @return how many balls reached the flywheel this loop (launch one shot each)
   */
  public int update(double spindexerRps, double feederRps) {
    double now = Timer.getFPGATimestamp();
    // 1. Balls that finished flying in are dropped into the hopper, still moving back and down.
    for (var it = incoming.iterator(); it.hasNext(); ) {
      Incoming b = it.next();
      if (now - b.startTime() < FEED_ANIMATION_SECS) continue;
      it.remove();
      Translation3d drop = dropPoint(b.start());
      Ball ball = new Ball(drop.getX(), drop.getY(), drop.getZ());
      ball.vx = -0.8;
      ball.vz = -0.5;
      hopper.add(ball);
    }

    // 2. Feeder: carry balls up, keeping a ball's width between them; the top one shoots.
    double feedSpeed = Math.max(0.0, -feederRps) * FEEDER_BALL_M_PER_ROTOR_REV;
    int shots = 0;
    for (int i = 0; i < feeder.size(); i++) {
      double limit = i == 0 ? Double.MAX_VALUE : feeder.get(i - 1) - FEEDER_SPACING_M;
      feeder.set(i, Math.min(feeder.get(i) + feedSpeed * LOOP_PERIOD_SECS, limit));
    }
    while (!feeder.isEmpty() && feeder.get(0) >= FEEDER_PATH_LENGTH_M) {
      feeder.remove(0);
      shots++;
    }

    // 3. Hopper physics.
    double spinRadPerSec = Math.abs(spindexerRps) * SPINDEXER_REV_PER_ROTOR_REV * 2 * Math.PI;
    double h = LOOP_PERIOD_SECS / SUBSTEPS;
    for (int s = 0; s < SUBSTEPS; s++) stepHopper(h, spinRadPerSec);

    // 4. The feeder takes a ball sitting at its entrance, if it's running and has room.
    boolean feederHasRoom = feedSpeed > 0
        && (feeder.isEmpty() || feeder.get(feeder.size() - 1) >= FEEDER_SPACING_M);
    if (feederHasRoom) {
      Ball closest = null;
      double best = Double.MAX_VALUE;
      for (Ball b : hopper) {
        if (!inFeederMouth(b)) continue;
        double d = Math.hypot(b.x - FEEDER_PATH[0].getX(), b.y - FEEDER_PATH[0].getY());
        if (d < best) {
          best = d;
          closest = b;
        }
      }
      if (closest != null) {
        hopper.remove(closest);
        feeder.add(0.0);
      }
    }
    return shots;
  }

  private void stepHopper(double h, double spinRadPerSec) {
    // Integrate: gravity, plus the spinning floor dragging the balls resting on it.
    for (Ball b : hopper) {
      b.vz -= GRAVITY * h;
      if (inFeederMouth(b)) {
        // Caught in the feeder's funnel: instead of riding around with the floor it's drawn in
        // toward the entrance, and waits there until the feeder has room (balls behind bunch up).
        double dx = FEEDER_PATH[0].getX() - b.x, dy = FEEDER_PATH[0].getY() - b.y;
        double d = Math.max(1e-6, Math.hypot(dx, dy));
        double blend = Math.min(1.0, SPINDEXER_GRIP_PER_SEC * h);
        b.vx += (FEEDER_FUNNEL_MPS * dx / d - b.vx) * blend;
        b.vy += (FEEDER_FUNNEL_MPS * dy / d - b.vy) * blend;
      } else if (b.onFloor && onSpindexer(b)) {
        double rx = b.x - SPINDEXER_CENTER.getX(), ry = b.y;
        double r = Math.hypot(rx, ry);
        // Friction pulls the ball toward the floor's velocity under it (ω × r)...
        double blend = Math.min(1.0, SPINDEXER_GRIP_PER_SEC * h);
        b.vx += (-spinRadPerSec * ry - b.vx) * blend;
        b.vy += (spinRadPerSec * rx - b.vy) * blend;
        // ...and the spin flings it out toward the edge, where the feeder entrance is.
        if (r > 1e-6 && spinRadPerSec > 0) {
          b.vx += SPINDEXER_FLING_MPS2 * rx / r * h;
          b.vy += SPINDEXER_FLING_MPS2 * ry / r * h;
        }
      }
      b.px = b.x;
      b.py = b.y;
      b.pz = b.z;
      b.x += b.vx * h;
      b.y += b.vy * h;
      b.z += b.vz * h;
    }

    // Solve: push overlapping balls apart, then keep everyone inside the hopper.
    for (int it = 0; it < SOLVER_ITERATIONS; it++) {
      for (int i = 0; i < hopper.size(); i++) {
        Ball a = hopper.get(i);
        for (int j = i + 1; j < hopper.size(); j++) {
          Ball c = hopper.get(j);
          double dx = c.x - a.x, dy = c.y - a.y, dz = c.z - a.z;
          double d2 = dx * dx + dy * dy + dz * dz;
          if (d2 >= 4 * R * R) continue;
          double d = Math.sqrt(d2);
          if (d < 1e-9) {
            dx = 1e-3;
            d = 1e-3;
          }
          double push = (2 * R - d) / 2 / d;
          a.x -= dx * push;
          a.y -= dy * push;
          a.z -= dz * push;
          c.x += dx * push;
          c.y += dy * push;
          c.z += dz * push;
        }
      }
      for (Ball b : hopper) keepInside(b);
    }

    // Velocity from how far each ball actually moved, with a little damping.
    for (Ball b : hopper) {
      b.vx = (b.x - b.px) / h * HOPPER_DAMPING;
      b.vy = (b.y - b.py) / h * HOPPER_DAMPING;
      b.vz = (b.z - b.pz) / h * HOPPER_DAMPING;
    }
  }

  /** Hopper walls, the floor (spinning floor + the ramp down to it in front), and the shooter. */
  private static void keepInside(Ball b) {
    b.x = Math.max(HOPPER_BACK_X_M + R, Math.min(HOPPER_FRONT_X_M - R, b.x));
    b.y = Math.max(-HOPPER_HALF_WIDTH_M + R, Math.min(HOPPER_HALF_WIDTH_M - R, b.y));
    // The feeder/shooter tower fills the back of the hopper above the spindexer.
    if (b.z > SHOOTER_BOTTOM_Z_M - R && b.x < SHOOTER_FRONT_X_M + R) b.x = SHOOTER_FRONT_X_M + R;

    double rx = b.x - SPINDEXER_CENTER.getX(), ry = b.y;
    double r = Math.hypot(rx, ry);
    double floor = floorZ(r);
    b.onFloor = b.z <= floor + 0.002;
    if (b.z < floor) {
      if (r > SPINDEXER_FLOOR_RADIUS_M) {
        // On the funnel around the spinning floor: push out along its surface normal, so balls
        // slide back down onto the spindexer instead of sitting in the hopper's corners.
        double l2 = 1 + HOPPER_RAMP_SLOPE * HOPPER_RAMP_SLOPE;
        double below = b.z - floor; // negative
        b.x += below * HOPPER_RAMP_SLOPE / l2 * rx / r;
        b.y += below * HOPPER_RAMP_SLOPE / l2 * ry / r;
        b.z -= below / l2;
      } else {
        b.z = floor;
      }
    }

    // Balls down on the spinning floor stay inside its rim.
    if (onSpindexer(b)) {
      double ox = b.x - SPINDEXER_CENTER.getX(), oy = b.y;
      double out = Math.hypot(ox, oy);
      if (out > SPINDEXER_RING_RADIUS_M) {
        b.x = SPINDEXER_CENTER.getX() + ox / out * SPINDEXER_RING_RADIUS_M;
        b.y = oy / out * SPINDEXER_RING_RADIUS_M;
      }
    }
  }

  /**
   * Ball-center height of the hopper floor at {@code r} from the spindexer center: flat on the
   * spinning floor, then a funnel rising all around it (toward the front, over the intake, and
   * into the hopper's corners).
   */
  private static double floorZ(double r) {
    return SPINDEXER_CENTER.getZ() + Math.max(0.0, r - SPINDEXER_FLOOR_RADIUS_M) * HOPPER_RAMP_SLOPE;
  }

  /** Low and within the feeder's mouth at the back edge of the spinning floor? */
  private static boolean inFeederMouth(Ball b) {
    Translation3d entrance = FEEDER_PATH[0];
    return b.z < entrance.getZ() + 0.05
        && Math.hypot(b.x - entrance.getX(), b.y - entrance.getY()) < FEEDER_CAPTURE_RADIUS_M;
  }

  /** Down on the spinning floor (not up in the pile, not out on the ramp)? */
  private static boolean onSpindexer(Ball b) {
    double r = Math.hypot(b.x - SPINDEXER_CENTER.getX(), b.y);
    return b.z < SPINDEXER_CENTER.getZ() + R && r < SPINDEXER_FLOOR_RADIUS_M;
  }

  /** Where a fed-in ball is let go: over the front of the hopper, at its pickup side. */
  private static Translation3d dropPoint(Translation3d pickup) {
    double y = Math.max(-HOPPER_HALF_WIDTH_M + R, Math.min(HOPPER_HALF_WIDTH_M - R, pickup.getY()));
    return new Translation3d(HOPPER_DROP_POINT.getX(), y, HOPPER_DROP_POINT.getZ());
  }

  /** Field poses of every ball inside the robot, for drawing. */
  public List<Pose3d> getPoses(Pose3d robotPose) {
    List<Translation3d> where = new ArrayList<>();
    double now = Timer.getFPGATimestamp();
    for (Incoming b : incoming) {
      double t = Math.min(1.0, (now - b.startTime()) / FEED_ANIMATION_SECS);
      where.add(arc(b.start(), FEED_ARC_OVER_ROLLER, dropPoint(b.start()), t));
    }
    for (Ball b : hopper) where.add(new Translation3d(b.x, b.y, b.z));
    for (double s : feeder) where.add(alongFeeder(s));

    List<Pose3d> poses = new ArrayList<>(where.size());
    for (Translation3d t : where) poses.add(robotPose.transformBy(new Transform3d(t, new Rotation3d())));
    return poses;
  }

  private static Translation3d alongFeeder(double s) {
    for (int i = 0; i < FEEDER_PATH.length - 1; i++) {
      double segment = FEEDER_PATH[i].getDistance(FEEDER_PATH[i + 1]);
      if (s <= segment) return FEEDER_PATH[i].interpolate(FEEDER_PATH[i + 1], s / segment);
      s -= segment;
    }
    return FEEDER_PATH[FEEDER_PATH.length - 1];
  }

  private static double pathLength() {
    double total = 0;
    for (int i = 0; i < FEEDER_PATH.length - 1; i++) total += FEEDER_PATH[i].getDistance(FEEDER_PATH[i + 1]);
    return total;
  }

  /** Quadratic Bezier from a through (toward) b to c at t in [0, 1]. */
  private static Translation3d arc(Translation3d a, Translation3d b, Translation3d c, double t) {
    double u = 1.0 - t;
    return a.times(u * u).plus(b.times(2 * u * t)).plus(c.times(t * t));
  }
}
