package frc.robot.sim;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.concurrent.atomic.AtomicBoolean;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Fuel on the field: rolling over the bumps, and being fed in by the intake. */
class FieldFuelTest {
  // Blue bump, lower lane: ascending face x 3.96→4.61 m (0→0.165 m up), descending to 5.18 m.
  private static final double LANE_Y = 2.5;

  @BeforeAll
  static void setupHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void freshArena() {
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SimulatedArena.getInstance().clearGamePieces();
  }

  private static void step(double seconds, Runnable each) {
    for (int i = 0; i < (int) (seconds / 0.02); i++) {
      each.run();
      SimulatedArena.getInstance().simulationPeriodic();
    }
  }

  private static RebuiltFuelOnField addFuel(double x, double y, double vx) {
    RebuiltFuelOnField fuel = new RebuiltFuelOnField(new Translation2d(x, y));
    SimulatedArena.getInstance().addGamePiece(fuel);
    fuel.setLinearVelocity(vx, 0);
    return fuel;
  }

  @Test
  void bumpSurfaceHeight() {
    assertEquals(0.0, FuelBumpSim.heightAt(3.5, LANE_Y), 1e-9, "floor before the bump");
    assertEquals(0.165, FuelBumpSim.heightAt(4.61, LANE_Y), 1e-3, "peak");
    assertEquals(0.0, FuelBumpSim.heightAt(4.3, 4.0), 1e-9, "the hub gap between the two bumps is flat");
    Pose3d onSlope = FuelBumpSim.onSurface(new Pose3d(4.285, LANE_Y, 0.075, new edu.wpi.first.math.geometry.Rotation3d()));
    assertEquals(0.075 + 0.0825, onSlope.getZ(), 1e-3, "drawn on top of the ramp, halfway up");
  }

  @Test
  void slowFuelRollsBackOffTheBump() {
    var fuel = addFuel(4.2, LANE_Y, 0.0); // sitting part way up the ascending face
    step(3.0, FuelBumpSim::update);
    assertTrue(fuel.getPoseOnField().getX() < 3.96,
        "should roll back down off the bump, ended at x=" + fuel.getPoseOnField().getX());
  }

  @Test
  void fastFuelRollsOverTheBump() {
    var fuel = addFuel(3.7, LANE_Y, 3.0); // well over the ~1.5 m/s a rolling ball needs
    step(3.0, FuelBumpSim::update);
    assertTrue(fuel.getPoseOnField().getX() > 5.18,
        "should make it over the bump, ended at x=" + fuel.getPoseOnField().getX());
  }

  @Test
  void intakeFeedsOnlyFromUnderTheRollersAtItsRate() {
    var chassis =
        new SwerveDriveSimulation(
            DriveTrainSimulationConfig.Default().withBumperSize(Inches.of(30), Inches.of(28)),
            new Pose2d(2.0, 6.0, Rotation2d.kZero));
    SimulatedArena.getInstance().addDriveTrainSimulation(chassis);
    AtomicBoolean active = new AtomicBoolean(false);
    FuelPathSim fuel = new FuelPathSim();
    IntakeSim intake = new IntakeSim(chassis, fuel, active::get);

    // 6 fuel under the rollers in a 2x3 grid (spaced wider than a ball so they don't shove each
    // other), and 5 off to the side where the intake can't reach.
    for (double dx : new double[] {0.46, 0.63}) {
      for (double dy : new double[] {-0.17, 0.0, 0.17}) addFuel(2.0 + dx, 6.0 + dy, 0);
    }
    for (int i = 0; i < 5; i++) addFuel(2.0, 6.0 + 0.7 + 0.16 * i, 0);

    step(0.5, intake::update);
    assertEquals(0, fuel.getCount(), "nothing comes in while the intake is off");

    active.set(true);
    step(0.3, intake::update);
    // 10 balls/s: after 0.3 s about 3 (one extra if the first comes in immediately).
    assertTrue(fuel.getCount() >= 2 && fuel.getCount() <= 4,
        "rate-limited pickup, got " + fuel.getCount());

    step(1.5, intake::update);
    assertEquals(6, fuel.getCount(), "all 6 under the rollers, none of the 5 beside");
    assertEquals(6, fuel.getPoses(new Pose3d()).size(), "every held ball is drawn");
  }
}
