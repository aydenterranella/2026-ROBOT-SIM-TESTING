package frc.robot.sim;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Drives a MapleSim chassis into the red alliance wall and checks that the deployed hopper
 * extension, not the bumper, is what stops it, while fuel still slides under the extension.
 */
class HopperSimTest {
  private static final double RED_WALL_X = 16.540988; // Arena2026Rebuilt border line
  private static final double BUMPER_HALF_LENGTH = Inches.of(15).in(edu.wpi.first.units.Units.Meters);
  private static final double HOPPER_FRONT_X = MechanismSimConstants.HOPPER_COLLISION_FRONT_X_M;

  private SwerveDriveSimulation chassis;
  private HopperSim hopper;

  @BeforeAll
  static void setupHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void setup() {
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SimulatedArena.getInstance().clearGamePieces();
    // y = 1.5 m keeps clear of the towers and trench walls near the red wall.
    chassis =
        new SwerveDriveSimulation(
            DriveTrainSimulationConfig.Default().withBumperSize(Inches.of(30), Inches.of(28)),
            new Pose2d(15.0, 1.5, Rotation2d.kZero));
    SimulatedArena.getInstance().addDriveTrainSimulation(chassis);
    hopper = new HopperSim(chassis);
  }

  @AfterEach
  void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private void enable() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /** Pushes the chassis toward +X for {@code seconds}, stepping the hopper and the arena. */
  private void driveIntoWall(double seconds) {
    for (int i = 0; i < (int) (seconds / 0.02); i++) {
      chassis.setRobotSpeeds(new ChassisSpeeds(2.0, 0.0, 0.0));
      hopper.update();
      SimulatedArena.getInstance().simulationPeriodic();
    }
  }

  @Test
  void staysInUntilEnabledThenDeploys() {
    hopper.update();
    assertEquals(0.0, hopper.getExtension(), 1e-9, "hopper should start slid in");

    enable();
    for (int i = 0; i < 50; i++) hopper.update();
    assertEquals(1.0, hopper.getExtension(), 1e-9, "hopper should be fully out after 1 s");

    disable();
    hopper.update();
    assertEquals(1.0, hopper.getExtension(), 1e-9, "hopper should stay out once deployed");
  }

  @Test
  void stowedHopperLetsBumperReachWall() {
    driveIntoWall(2.0);
    double frontOfBumper = chassis.getSimulatedDriveTrainPose().getX() + BUMPER_HALF_LENGTH;
    assertEquals(RED_WALL_X, frontOfBumper, 0.05, "bumper should be against the wall");
  }

  @Test
  void deployedHopperHitsWallFirst() {
    enable();
    driveIntoWall(2.0);
    double frontOfHopper = chassis.getSimulatedDriveTrainPose().getX() + HOPPER_FRONT_X;
    assertEquals(RED_WALL_X, frontOfHopper, 0.05, "hopper front should be against the wall");
  }

  @Test
  void fuelPassesUnderDeployedHopper() {
    enable();
    for (int i = 0; i < 50; i++) hopper.update();
    // Fuel sitting in the middle of the extension's collision box.
    Translation2d spot =
        chassis.getSimulatedDriveTrainPose().getTranslation()
            .plus(new Translation2d(MechanismSimConstants.HOPPER_COLLISION_CENTER_X_M, 0.0));
    RebuiltFuelOnField fuel = new RebuiltFuelOnField(spot);
    SimulatedArena.getInstance().addGamePiece(fuel);
    for (int i = 0; i < 10; i++) {
      hopper.update();
      SimulatedArena.getInstance().simulationPeriodic();
    }
    assertEquals(0.0, fuel.getPoseOnField().getTranslation().getDistance(spot), 0.01,
        "fuel under the hopper extension shouldn't be shoved out");
  }
}
