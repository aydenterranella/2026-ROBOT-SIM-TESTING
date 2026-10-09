package frc.robot.sim;

import static frc.robot.sim.MechanismSimConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Sim-only hopper extension. The robot starts a match with the hopper slid in (inside the frame
 * perimeter); the first time the robot is enabled it slides out over the intake and stays out.
 *
 * <p>Once it's out, the extension is added to the MapleSim chassis as an extra collision box, so it
 * hits walls, the hub, and other robots. Fuel is filtered out of that box: the real extension is
 * ~10" off the floor and balls roll underneath it to the intake.
 */
public class HopperSim {
  // dyn4j collision categories. Fixtures MapleSim creates use the default filter, which collides
  // with everything, so only the hopper and fuel need categories.
  private static final long HOPPER_CATEGORY = 1L << 1;
  private static final long FUEL_CATEGORY = 1L << 2;
  private static final CategoryFilter HOPPER_FILTER =
      new CategoryFilter(HOPPER_CATEGORY, ~FUEL_CATEGORY);
  private static final CategoryFilter FUEL_FILTER =
      new CategoryFilter(FUEL_CATEGORY, ~HOPPER_CATEGORY);

  private final AbstractDriveTrainSimulation chassis;
  private final BodyFixture extensionFixture;
  private boolean deployed = false;
  private boolean fixtureAttached = false;
  /** 0 = fully in, 1 = fully out. */
  private double extension = 0.0;

  public HopperSim(AbstractDriveTrainSimulation chassis) {
    this.chassis = chassis;
    var box = Geometry.createRectangle(HOPPER_COLLISION_LENGTH_M, HOPPER_COLLISION_WIDTH_M);
    box.translate(new Vector2(HOPPER_COLLISION_CENTER_X_M, 0.0));
    extensionFixture = new BodyFixture(box);
    // Density 0 so adding the box doesn't change the chassis mass MapleSim computed.
    extensionFixture.setDensity(0.0);
    extensionFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
    extensionFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
    extensionFixture.setFilter(HOPPER_FILTER);
  }

  /** Call once per {@code simulationPeriodic()}, before the arena steps. */
  public void update() {
    if (DriverStation.isEnabled()) deployed = true;

    double step = LOOP_PERIOD_SECS / HOPPER_DEPLOY_SECS;
    extension = deployed ? Math.min(1.0, extension + step) : Math.max(0.0, extension - step);

    // The extension only sticks out past the bumpers once it's most of the way out.
    boolean shouldCollide = extension > 0.5;
    if (shouldCollide && !fixtureAttached) {
      chassis.addFixture(extensionFixture);
      fixtureAttached = true;
    } else if (!shouldCollide && fixtureAttached) {
      chassis.removeFixture(extensionFixture);
      fixtureAttached = false;
    }

    // Fuel can be spawned at any time (resets, shots landing), so tag any new pieces.
    for (GamePieceOnFieldSimulation piece : SimulatedArena.getInstance().gamePiecesOnField()) {
      for (BodyFixture fixture : piece.getFixtures()) {
        if (fixture.getFilter() != FUEL_FILTER) fixture.setFilter(FUEL_FILTER);
      }
    }
  }

  /** How far the extension has slid out, 0 (in) to 1 (out). */
  public double getExtension() {
    return extension;
  }
}
