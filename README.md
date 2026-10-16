# SOTM-POWERSIM

A copy of `2026-Robot-Powerlib-Rewrite` with the MapleSim simulation from
`2026-ROBOT-SOTM-REWRITE` underneath it. The robot code is `sotm-testing` at `365905bb`
(2026-10-16) merged with Chad's `codex/improved-auto-path-building` at `99f52e26`
(2026-10-15, continuous auto paths, bump-crossing command, single-tag heading sync). That
merge exists only here; no branch upstream has it yet. Not a git repo. The sim answers one
question: **how close does the sim feel to the real robot?**

**The merge:** two conflicts, both resolved without changing behavior on either side.
- **`RobotAutos.java`:** keeps Chad's settled final-aim drive in place of the old
  turn-until-aligned step, and keeps `sotm-testing`'s `aimAtHub` (shot-solution heading,
  parking). `aligned()` is gone because nothing calls it.
- **`StateMachineTest.java`:** both branches added a "wait for the sim drivetrain to stop"
  step between tests; this keeps `sotm-testing`'s `settleDrivetrain()`.

The merged robot code passed its own 107 tests on 4 runs out of 4 before the sim went
underneath. See "Autonomous in sim" below for what the sim found.

**Pulling new robot commits:** apart from the sim changes listed below, every file is
byte-identical to that merge, except the constants files Power Tool regenerated on
2026-10-16 (about 60 constants marked `@TunableConstant volatile`, same values). To catch up,
apply `git diff <old commit>..<new commit> -- . ':!README.md'` from the rewrite repo here
with `git apply`, and merge its README changes into the "Shoot on the move" section and
below (that part is upstream's README verbatim). Rebuild Power Tool
(`cd power-tool; npm run build`) if `power-tool/` changed, and update the commits above.

## Simulation

### Run it

```powershell
./gradlew.bat simulateJava
```

`gradle.properties` pins Gradle to the WPILib JDK (`C:\Users\Public\wpilib\2026\jdk`); the
system Java 26 fails with `Type T not present`. In the Sim GUI, drag your controller onto
Joystick 0, pick an alliance, then Teleoperated. Same buttons as the robot: left trigger
collects, right trigger shoots, B ejects.

**Power Tool:** `cd power-tool; npm start`. It defaults to **Local simulation**
(`localhost:5810`, File → Connection Settings → Target) and reads this folder's constants.
Do **not** use File → Update PowerLib Library Files here: it overwrites the
`PowerSubsystem` sim switch described below. Writing tuned values or tunable flags back to
code runs `gradlew --no-daemon build` afterwards, which includes the sim tests (about 1.5 min).
Let it finish, then restart `simulateJava` so the sim runs the new code. Live tuning (changing
a `@TunableConstant` in Power Tool while the sim runs) needs no rebuild.

**AdvantageScope:** the `9410 2026 Robot` model is already in
`%APPDATA%\AdvantageScope\userAssets\Robot_9410_2026` (a copy is in `advantagescope/`).
File → Connect to Simulator, add a 3D Field, drag `MapleSim/RobotPose3d` in as the robot,
`Sim/Mechanisms/ComponentPoses` onto it as components, and `MapleSim/Fuel3d` as game pieces.
Restart AdvantageScope after the model files change.
The full guide is section 1 of the `2026-ROBOT-SOTM-REWRITE` README.

### What changed from the robot code

| File | Change |
|---|---|
| `src/main/java/frc/robot/sim/*` | The SOTM rewrite's sim package: flywheel/feeder/spindexer/intake-roller physics (`DCMotorSim`, inertia from SysId kA), intake wrist and hood arms (`SingleJointedArmSim` with gravity and hard stops), hopper extension, fuel path through the robot, intake pickup, shot launch, bumps. Ported to PowerLib constants; numbers unchanged. One change from the vendored bump sim (`RobotBumpSim`): its ramp was frictionless, so a driven robot could only coast over and anything under ~1.8 m/s slid back. Now the wheels keep pushing on the ramp (MapleSim's wheel-force speed change is added to gravity), so the robot drives over the bump the way Chad's bump-crossing command expects. Coasting still behaves as before. Two later fixes: hopper balls are capped at 2.5 m/s (0.8 m/s upward), because the solver turned overlap pushes into 10-30 m/s and launched balls metres up while shooting (the feed rate is unchanged); and roller friction can stop a roller but not reverse it, because a stopped intake roller used to flip between ±0.9 rps every loop, which made its Power Tool tile change sign. |
| `frc/powerlib/subsystems/PowerSubsystem.java` (+ the four subsystem types) | New `PowerSubsystem.usePhysicsSimulation(true)`. By default PowerLib builds no TalonFX in sim and uses a kinematic `*IOSim`, so PID, Motion Magic and flywheel spin-up never really happen. With the switch on, sim builds the real devices and the real IO path, and the physics models drive the TalonFX/CANcoder sim states. Off by default, so the existing tests are unchanged. |
| `Robot.java` | Flips that switch, swaps in the 2026 arena (bump colliders off, full fuel load) before `RobotContainer`, and runs the SOTM sim loop in `simulationInit()` / `simulationPeriodic()`. Intake pickup is gated on the arm being down and the rollers spinning inward (this code can collect while spinning up or shooting, not only in `INTAKING`). If no alliance has been reported at launch, it sets Blue 1: the Sim GUI starts "Disconnected" with no alliance, so the alliance-filtered auto chooser offered only "None" until the robot had been enabled once. The GUI copies the station into its Alliance Station box, so switch to Red there. |
| `Swerve.java` | MapleSim drivetrain (55 kg, 30"×28" bumpers, X60s (FOC model on drive), 5.4:1 / 12.1:1, 0.01 kg·m² steer inertia, Colsons, 0.1 /s body drag) replaces CTRE's 5 ms sim notifier. MapleSim drives the wheels from the voltages Phoenix commands, using the robot's real steer and drive gains; `PhoenixSimUtil.regulateForSimulation` only zeroes encoder offsets, inversions and the steer-drive coupling ratio. Odometry is synced to MapleSim's ground truth each loop. |
| `build.gradle` | `forkEvery = 1` for tests (HAL, arena and the sim switch are global). The auto route tests are left out of `test` and run with `./gradlew autoSim`. |
| `src/test/java/frc/robot/sim/*` | Sim tests from the SOTM rewrite, ported. `RobotSimSmokeTest` builds the real `Robot` and checks spawn, hopper, intake, and that holding shoot spins up, aims, fires and scores while stationary. `SimShotTest` fires every shot-table row. `SwerveSteerSimTest` steps the wheels 85° and to X-lock on the real steer values and checks they settle and hold without buzzing. `StationaryAimXLockSimTest` (SOTM off) and `StationaryAimParallelSimTest` (SOTM on) hold shoot from 4.2 m and check the aim target holds still while turning, the wheels park once, and they stay parked without sliding while it fires. `Auto*SimTest` (one per route, `./gradlew autoSim`) runs each auto end to end; see "Autonomous in sim". Every test robot is built through `SimTestRobots`, which keeps NetworkTables local so a test run never takes ports 5810/1735 away from a sim you have open. |

### Where the sim is not the robot

Compare these on the robot before trusting the sim for them:

- **Steering.** The sim runs the robot's own steer values (kP 50, 12.1:1, fused CANcoder,
  60 A limit). It only zeroes encoder offsets and inversions, because MapleSim starts every
  wheel at zero and the robot's offsets would make the wheels vibrate in place, and the
  coupling ratio (4.5), because MapleSim's steering doesn't turn the drive wheel the way the
  module gears do. With the ratio left in, Phoenix read every steer move as wheel speed, so the
  stationary X-lock kept letting go and re-locking and the wheels whirled while aiming. Two
  gaps remain:
  - Phoenix's simulated motor controllers run on the wall clock, so they see each MapleSim step
    once per 20 ms loop and react a loop or two late. Big steer moves overshoot more than on
    the robot: an 85° step overshoots ~32° and settles in ~0.3 s, where a model of the robot's
    1 kHz loop predicts ~0.1 s with no overshoot. 30–45° moves (X-lock, parking) settle in
    0.1–0.15 s, close to the robot, and nothing buzzes at rest.
  - The MapleSim steer inertia (0.01 kg·m² in `Swerve.initMapleSim`) is an ESTIMATE; no value
    is published for this module. It sets how fast the wheels turn and park. Log a steer step
    on the robot (X-lock from straight is easy) and adjust it until the sim's settle time
    matches; don't go below 0.007, where MapleSim's integrator starts to ring.
- **Drive speed.** The drive gains are feedforward only (kP 0, kS 0), so how closely speed
  tracks depends on the motor model and drag. MapleSim's swerve drivetrain hardcodes a 1.4 /s
  body drag, which held 1.5 m/s requests to 1.31 m/s (13% short at every speed). The sim now
  uses 0.1 (MapleSim's base default, an ESTIMATE) and the Kraken X60 FOC model the robot's
  drive actually runs, so 1.5 m/s reaches ~1.45 and 3.0 reaches ~2.92. The remaining ~3% is
  kV 0.122 sitting a bit under what an FOC X60 needs; the real robot also loses a little to
  friction (kS was never measured), so expect it near 1.40-1.46 m/s at a 1.5 request. To
  calibrate, log applied drive volts V and speed v during a steady cruise and set
  `setLinearDamping` in `Swerve.initMapleSim` to `((V - 0.037) / v - 2.091) / 0.1656`. If that
  comes out near 1.4, the robot really does run 13% slow and needs drive kP. Turning has the
  same hardcoded 1.4 drag (`setAngularDamping`), left as is and untested; rotation requests
  are likely ~6% slow in sim.
- **Gear ratios (checked against the Onshape CAD on 2026-10-16).**
  - **Hood motor gearing:** the CAD gear train is Kraken X44 → 12t:48t → 10t pinion on the 190t
    sectors = **76:1**. The sim used to derive 115:1 from the code's
    `ShooterHoodConstants.ROTOR_TO_SENSOR_RATIO = 60`. The sim's physics now uses 76
    (`HOOD_ROTOR_PER_ARM_ROT`).
  - **Robot-code issue, for the team:** that 60 matches the old turret-era shooter (12:48, then
    10:150). With the CANcoder geared 46:24 off the hood, the real rotor-to-CANcoder ratio is
    76 × 24/46 = **39.65**, so the robot's fused hood velocity reads about 1.5× off. The robot
    code here is left as is.
  - **Hood geometry and travel:** the CANcoder gearing (46:24), the pivot, the 22.1° rest angle
    and the swing direction all match the CAD; the hood swings 8.6° across the shot range. The
    sector teeth allow only ~20–22° above rest, so the sim's top stop is now 0.11 code rotations.
  - **Swerve:** the CAD's module setting says X3 10t (6.48:1), while the code uses 5.4:1 (X3
    12t). Kept 5.4, since odometry would be ~20% off on the robot if it were wrong. The CAD is
    probably stale.
  - **Not settled from CAD:**
    - The spindexer floor ratio: the MAX 90° box ratio isn't published.
    - The feeder's motors and roller travel: `FEEDER_BALL_M_PER_ROTOR_REV` is a fire-rate knob.
    - The intake roller's inertia.
    - The intake's stow angle.
  - **AdvantageScope model, cleaned 2026-10-16:**
    - **Removed:** a duplicate X60 that sat 0.14" from the right flywheel motor and flickered;
      the 36-part turret gearbox left under the shooter (gears, encoder, shafts, bearings,
      spacers), which nothing drove, while its plates and nut strips stay; and the stray X44 on
      the intake arm (the roller is an X60, per the team).
    - **Rerun after a new CAD export:** run `split-robot-glb.mjs`, then
      `node tools/advantagescope/prune-glb.mjs <in.glb> <out.glb> <rules>` with
      `prune-rules-model.json` for `model.glb` and `prune-rules-model_0.json` for `model_0.glb`
      (pass `--dry-run` in place of `<out.glb>` to list matches only).
    - **Originals:** in `advantagescope/backup-2026-10-16-before-cleanup/` on the desktop PC
      only; it's gitignored to keep ~100 MB out of the repo. To restore, copy them into both
      `advantagescope/Robot_9410_2026/` and AdvantageScope's assets folder.
- **Vision.** None in sim; odometry is ground truth. SOTM confidence reads 0.5 instead of 1.
- **Fire rate.** `FEEDER_BALL_M_PER_ROTOR_REV` (ESTIMATE in `MechanismSimConstants`) was set
  so the SOTM rewrite's ~110 rps feeder fired ~8 balls/s. This code runs the feeder at 2×
  flywheel demand (~60 rps), which works out to ~5 balls/s in sim (computed, not timed). Count
  balls per second on the robot and scale that constant to match.
- **Mass and bumpers** (55 kg, 30"×28") and every value marked ESTIMATE in
  `MechanismSimConstants` (intake rate, spindexer grip, arm inertias, hopper slide time).
- **Shots.** See below.

### Shot model

The sim launches each ball from the *simulated* flywheel speed and hood position, so a shot
taken before they settle misses. The launch model (`MechanismSimConstants`, "Shots") is fit to
this robot's shot table and the setpoints the code commands (table + 1 rps, hood - 0.005):

```
exit speed (m/s)   = 0.220 × flywheel rotor rps
launch angle (deg) = 65.5 - 131.5 × hood position (rotations)
```

The SOTM rewrite's model (0.0986 / 58.81 / -50.37) was fit to the older turret-era shooter
(62-89 rps, 2026-Robot `f155723`). This code uses the April 4 `NewShooterTesting` shooter
(27-37 rps, 2026-Robot `e266cd4` / `0f75d1b`), where the old model falls about 3 m short.

| Distance (m) | Commanded rps | Commanded hood | Exit (m/s) | Angle (°) | Flight time (s) |
|---|---|---|---|---|---|
| 1.95 | 28.4 | 0.039 | 6.25 | 60.4 | 0.57 |
| 2.50 | 29.5 | 0.050 | 6.49 | 58.9 | 0.69 |
| 3.00 | 30.5 | 0.060 | 6.71 | 57.6 | 0.78 |
| 3.50 | 33.0 | 0.065 | 7.26 | 57.0 | 0.83 |
| 4.10 | 35.0 | 0.075 | 7.70 | 55.6 | 0.90 |
| 4.90 | 38.0 | 0.085 | 8.36 | 54.3 | 0.96 |

`SimShotTest` checks every row scores and an under-speed shot misses. `RobotSimSmokeTest`
holds shoot from 2.8 m with the real code (aim, spin-up, feed) and scores 10 of 10.

Caveats:

- **Flight times are not measured.** The table's `SHOOTER_TOF` is provisional (copied from the
  SOTM rewrite), and the angle terms lean on it. The sim's flight times above agree with it
  within 2%. Time a few shots on video; if they differ, re-fit the angle terms.
- **Exit speed looks high for one 4" wheel**: the CAD belts the motors 18t:24t, so the wheel
  turns 0.75× the rotor, and 0.220 m/s per rotor rps is ~92% of its surface speed (a single
  wheel on a fixed hood is usually under 50%). The hood's driven upper rollers probably make up
  the difference; worth confirming with the flight-time video.
- **Moving shots lead short in sim.** MapleSim's ball has no drag, but the SOTM solver assumes
  `SOTM_DRAG_COEFFICIENT` = 0.24, so in sim the lead comes out 7-11% short. Set it to 0 to test
  the solver's geometry in sim; leave it alone on the robot.
- **Launch point** (0.20 m behind center, 0.57 m up) is carried over from the SOTM rewrite and
  not checked against the new shooter's CAD.
- **MapleSim counts a ball when its center enters a 47" box at the hub's height**, with no check
  that it is falling, so sim scoring is more forgiving than the real hub.

### Autonomous in sim

```powershell
./gradlew.bat autoSim                                           # all four routes, ~1.5 min
./gradlew.bat autoSim --tests frc.robot.sim.AutoBlueLeftSimTest # one route
```

Each test places the robot on the route's start pose with 8 preloaded fuel while disabled,
picks the route in the dashboard's Auto Chooser, and runs the full 15 s auto on the real
robot code. It fails if the auto aborts early, misses a waypoint, collects nothing, or scores
less than the preload. The output logs each waypoint's closest approach and when it was
reached, state changes, and the pose and speed every 0.1 s.

Results on 2026-10-16, both with the same sim (bump fix, drive model, coupling fix):

| Route | `sotm-testing` autos (before the merge) | With Chad's auto branch (this folder) |
|---|---|---|
| Blue Left | Pass: all 7 waypoints by 6.8 s, scored 25 | Fail: aborted at 14.0 s on the return leg, 26 collected, 0 scored |
| Blue Right | Pass: scored 28 | Fail: aborted at 7.1 s, between waypoints 4 and 5 |
| Red Left | Pass: scored 26 | Fail: aborted at 7.1 s, between waypoints 4 and 5 |
| Red Right | Pass: scored 23 | Fail: aborted at 3.0 s at waypoint 2 |

Why Chad's version aborts in sim (the per-waypoint 3 s deadline runs out each time):

- **Corners are taken at full planned speed.** `PathProgress` plans the speed for passing an
  intermediate waypoint from the next leg's length and speed limit, not from how sharp the
  turn is (his README lists corner smoothing as later work). Waypoint 3 is crossed at about
  2.6 m/s heading toward the side wall, followed by a ~100° turn. Stopping that at the 3 m/s²
  chassis limit takes about 1 m, but there is about 0.4 m of room, so the sim robot hits the
  wall and loses the time. Waypoint 6 is crossed at about 2.5 m/s and then turns 90° onto the
  6 m return leg. The robot swings about 2 m wide into the trench lane and is still 1 m off
  line when that leg's 3 s runs out. Both come from the code's own limits, so expect the same
  on the robot.
- **The 6 m return leg has almost no slack.** Even with perfect tracking, 6 m at 3 m/s²
  takes 2.8 s, against a 3 s deadline.
- **Stopping at waypoint 2 is marginal.** That leg runs open loop (heading not locked). The
  robot comes off the bump at about 2.9 m/s, carries about 0.2 m past the 6" tolerance, and
  has to creep back before the settle check passes. Blue Left made it at 2.98 s; Red Right
  missed at 3.04 s. How hard the robot brakes open loop depends on drag and motor braking,
  so this one is the most sim-sensitive.

The `sotm-testing` autos stop at every waypoint, so corners don't matter. They finish the
route by about 6.8 s and start shooting at about 8.9 s. They do carry about 1 m past the last
waypoint before aiming (Blue Left ends at x = 1.05 instead of 2.10), which Chad's settled
arrival is meant to fix.

## Shoot on the move (`sotm-testing`)

Holding shoot now keeps driver translation available at the same speed as intaking
(`DRIVE_WHILE_INTAKING_COEFFICIENT`, 40%). The runtime firecontrol classes `ShotCalculator`,
`ShotLUT` and `ShotParameters` were adapted from
[2026-ROBOT-SIM-TESTING](https://github.com/aydenterranella/2026-ROBOT-SIM-TESTING/tree/e2bfadbd89e17429873ec029ce2a8b123377f4ec/src/main/java/frc/firecontrol).
Their FRC Team 5962 perSEVERE MIT attribution is retained. No MapleSim, projectile
simulator, LUT generator, or simulation-only dependencies were added.

The solver converts measured robot-relative velocity into field coordinates, low-pass
filters it (`SOTM_VELOCITY_FILTER_SECONDS`, 0.1 s), predicts the launcher through a
20 ms release delay, includes rotational velocity at the rear-mounted launcher, and
solves a drag-adjusted virtual target using Newton's method. Acceleration is not
estimated: differencing measured velocity roughly tripled noise in the aim heading.
Current RPS/hood tables and offsets remain intact: robot-center calibration distances
are shifted by the 0.20 m rear launcher offset inside the lookup table so stationary
shots retain their original demands. Phoenix already supplies a current pose, so no
additional vision latency is applied.

The imported flight-time table is **provisional**: the reference used different RPS
and hood settings. Measure flight times for this robot's retained calibration before
relying on moving-shot accuracy. Verify the 0.20 m launcher offset and release delay
on hardware too. No new on-robot shot calibration is claimed by the software tests.

When the robot isn't driving (filtered translation under 0.1 m/s, including turning in
place to aim), it uses the original standing shot unchanged, and tilt is not checked.
While driving, both actual center distance and compensated calibration distance must
remain within 1.95-4.9 m in the scoring zone. Feeding still requires the existing
5-degree heading, hood, flywheel and feedback checks. Excessive tilt (over the tunable
`SOTM_MAX_TILT_DEGREES`, 5 degrees), speed (over 3 m/s), invalid input, uncalibrated lead
distance or solver nonconvergence prevents a moving shot and stops the shooter until a valid shot returns. Solver
confidence is diagnostic; feed permission uses the explicit validity and readiness checks.

Teleop aiming slews the heading target at up to `AIM_HEADING_SLEW_DEGREES_PER_SECOND`
(90) so lead changes and moving/static fallbacks turn the robot smoothly, and adds the
configured `Heading/kD` as damping. Lower the slew rate if the wheels still splay while
aiming. Once translation and rotation settle within 1 degree of aim, wheels park
instead of retaining the turn-in-place stance: parallel with SOTM on, or X-locked
(`SwerveDriveBrake`) with SOTM off so the stationary shot resists pushes. Driver
translation or drift beyond 2 degrees resumes normal aiming. This applies to teleop and
the autonomous final aim. The turn-in-place wheel angles remain necessary while the
robot is rotating.

The **Shoot on move** button on Power Tool's Shot ready card is the kill switch. OFF
restores the original stationary shot with driver translation held at zero while
aiming. The button writes `/PowerLib/Shooting/SOTMRequestedEnabled`; `SotmSwitch`
applies each new request every loop, with or without tuning, and the button shows the
robot's reported `Shooting/SOTMEnabled`. Robot code starts with
`SOTM_ENABLED_DEFAULT` (on).

The Shot ready card also shows SOTM confidence, lead angle and flight time, and the
compensated aim distance. The robot publishes
`Shooting/SOTMEnabled`, `LeadDegrees`, `EffectiveDistanceMeters`, `TimeOfFlightSeconds`,
`MotionCompensated`, `SOTMValid`, `SOTMConfidence`, `SolutionStatus`,
`Drive/AimWheelsParked` and `Drive/AimWheelsLocked`. Tunable constants include
`RobotContainer/Custom/DRIVE_WHILE_INTAKING_COEFFICIENT` (intake and shooting drive speed)
and Shooting's `SOTM_MAX_DRIVETRAIN_SPEED_MPS`, `SOTM_RELEASE_DELAY_SECONDS`, `SOTM_DRAG_COEFFICIENT`,
`SOTM_MAX_TILT_DEGREES`, `SOTM_VELOCITY_FILTER_SECONDS` and
`AIM_HEADING_SLEW_DEGREES_PER_SECOND`. Source
constants and `powerlib-constants.json` are kept in sync. Tests cover velocity lead,
filtering, release latency, rotating-launcher velocity, solver failures, retained
stationary calibrations, compensated readiness, the kill switch, intake-speed driving, aim
slewing and parking.

## PowerLib installation

This branch starts from the fresh `main` robot project at `c37a98f33`.
PowerLib source and Power Tool were installed from Robot-Library commit
`5f73c86bbc72bbc5c4092657fabae3199e848722` with all options enabled:
library, robot templates, vendor dependencies, generator/tools, Electron dashboard,
and simulation skills. Robot and Power Tool production builds passed.

Robot configuration uses original 2026-Robot commit
`0f75d1b75d1428dd8b73213b2447adb292ab2125` (April 4, 2026, on
`NewShooterTesting`). Seven mechanisms, swerve settings, shooting tables, field
geometry and all 28 auto waypoints were compared against that commit. Power Tool
configuration is saved in `power-tool/generated/powerlib-subsystems.json` and
`power-tool/generated/powerlib-constants.json`; regenerate with
`power-tool/scripts/generate-subsystem.ps1 -UpdateSubsystems`.

The four autos use the original intake/return/turn/shoot sequence, per-leg speeds,
tolerances and heading-lock choices. Red headings are converted to blue-field
coordinates. Auto selection follows the reported alliance, with `None` as the default.
The rewrite retains 100 ms dashboard updates, 200 ms tuning, 50 Hz logging,
readiness/freshness checks, disabled-only pose seeding, start-pose guards and auto
leg timeouts. Shooting allows distances from 1.95-4.9 m. A new 1.95 m row extends
the closest April 4 segment (2.5-3.0 m): 27.4 RPS and 0.044 hood rotations before
offsets. Shots closer than 1.95 m are rejected. The new values are extrapolated and need
hardware tuning. `gradlew build` tests shot outputs, route values,
state transitions, cancellation and timeout behavior.

Motor configuration was audited again against April 4 commit `0f75d1b`. Shooter
leader CAN 51 uses `reversed: false` in PowerLib to preserve the legacy
CounterClockwise_Positive direction; followers 52 and 53 remain opposed. The other
active mechanism leaders already use that direction. Feeder, shooter and spindexer
also restore the legacy 140 A stator current limit on their leaders, explicitly
saved in the subsystem JSON and emitted by the generator. PowerLib preserves the
declared neutral modes; legacy full configuration writes could reset those modes.
RegionalMotorConfigurationTest checks the Phoenix output configuration without
hardware. Physical direction still needs verification on the robot.

The audit includes feeder, intake roller/wrist, shooter/hood and spindexer CAN IDs,
follower alignment, PID/feedforward, feedback ratios, encoder offsets, Motion Magic
settings and position defaults/limits. Shared auto waypoints, Tuner hardware,
controller, LED and camera constants retain the April 4 values. Field dimensions
are corrected to 16.541 by 8.069 m and hub centers to red (11.915521, 4.0346376)
and blue (4.6255178, 4.0346376), derived from the WPILib 2026 welded layout's opposing
hub-face tags. The red zone ends at the corrected field length. The screenshot pose
(13.27, 3.18) is about 1.602 m from the corrected red hub, below the retained 1.95 m
minimum shooting distance; its target heading is about -32.25 degrees. The removed turret
and legacy sweep feature are not reinstated; their active shooter/hood tables live
in ShootingConstants. New PowerLib vision/readiness settings remain rewrite-specific.

State handlers implement Idle/Ready, Intaking, Ejecting, Spinning Up, and Shooting.
Left trigger collects, B requests eject, and right trigger requests shooting.
A shooting request spins up first and feeds only when shooter velocity, hood position,
heading, shot range, and feedback are ready. Losing readiness immediately stops feeding.
Releasing B resumes a held shooting or intake trigger; releasing all triggers returns
to `IDLE`. Intake can run while shooting. Shared geometry, interpolation,
readiness, and shooter/feeder actions live in `ShootingUtils`. Values and driver behavior
use the April 4 calibrations described above. Back seeds fresh vision position while disabled
and uses fresh independent multi-tag heading when available, preserving heading otherwise;
Start zeros heading while disabled, preserving X/Y. Test mode leaves outputs to SysId.

Each state request has an owner: `requestState(state, owner)` updates only that owner's
entry, and `clearRequest(owner)` removes it. The ordered handler list in `StateMachine`
is the sole request priority: Ejecting, Shooting, Spinning Up, Intaking, then Idle/Ready.
ButtonBindings.bindStates(button, pressState, releaseState, stateMachine::requestState)
creates a stable owner per binding. Both triggers and B request IDLE on release, so an
older button release cannot overwrite another held button's request. The function
helper is ButtonBindings.bindFunctions(button, onPress, onRelease).

HealthChecks in PowerLib owns signal freshness, finite-pose checks, and mechanism/drivetrain
feedback checks. Vision, drive commands, and shooting readiness use it. Robot code chooses
the mechanisms required for shooting. IntakingState owns collection and wrist travel limits;
EjectingState owns reverse intake outputs; IdleState owns roller stops and idle posture.
ShootingState owns the shot snapshot, required-mechanism health checks, and wrist-pulse
timer, shared with SpinningUpState. It delegates held collection to IntakingState.
StateMachine calls each handler's optional prepare hook before selection and reset hook
on mode changes. Shooting preparation runs only while a shooting or spin-up request is
pending; execution rechecks readiness every cycle. PowerDashboard publishes distance
independently, so it continues updating during idle and intake. Shared shooting utilities
contain no intake, wrist, or health behavior.

Heading corrections track independent MT1 frame timestamps. When an MT1 frame
arrives behind an already-applied MT2 position frame, its capture-time heading
error is carried forward to the latest fusion timestamp, preserving intervening
rotation and newer position corrections. Cameras remain unthrottled while disabled
to keep reset frames fresh. SignalLogger records Vision/Heading/Status, Accepted,
ErrorDegrees, PigeonYawDegrees and EstimatedDegrees. While disabled, fresh multi-tag
MT1 frames automatically initialize heading once per disabled period after at least
five distinct frames agree within 2 degrees for 0.25 s. Speed must be at most
0.05 m/s and yaw rate at most 5 degrees/s, with fresh drivetrain feedback. This
heading-only reset preserves X/Y and can recover errors larger than 15 degrees.
Vision/HeadingStatus and Vision/MT1HeadingFresh are also published to Power Tool.
While enabled, automatic correction still
requires two reliable tags within 4 m, frame age at most 0.2 s, yaw rate at most
45 degrees/s, and heading error at most 15 degrees. Back can reseed a larger heading
error from fresh independent heading while disabled; Start zeros blue-field heading.

The drive command receives live current state for intake/shooting speed scaling and
hub alignment. Robot tests cover shot-table values, readiness, transitions,
and button priority. PowerLib feedback-ratio validation also accepts signed CTRE
ratios, retaining the rewrite's negative sensor-to-mechanism ratios.

Autonomous stopping moves request braking before the tolerance boundary and require
position, optional heading, and low measured translation/angular speed to hold continuously.
The provisional arrival limits are 0.05 m/s, 5 degrees/s and 0.1 s, configured in AutoConstants;
validate them and route timing on hardware. The April 4 waypoints, per-leg tolerances and
heading locks are unchanged. Normal braking uses the shared chassis motion limiter;
cancellation and faults stop immediately. Leg deadlines remain 3 s, final aiming 1 s and
the overall routine 20 s. Final aiming uses the same completion check with a live hub heading
and the last waypoint's position tolerance; a timeout aborts before requesting SHOOTING.
The command caches completion during execute(), so isFinished() has no control side effects.

Each regional auto now uses three continuous driving sections: points 1-2 before INTAKING,
points 3-5 before READY, and points 6-7 before final aiming. DrivePathCommand retains its
distance controller, heading history and chassis limiter across intermediate waypoints;
DriveToPointCommand is the single-point wrapper around that same implementation. Intermediate
points advance on position tolerance, while section endpoints require measured-speed settling.
PathProgress caches braking bounds for lower future speed targets and the final stop. Per-leg
settings use ordered indices, so duplicate coordinates retain their own policies. The active
waypoint's 3 s deadline restarts only on advancement; timeout is checked before advancement.
Velocity transitions stay acceleration-limited. Intermediate handoffs accept radius arrival
or bounded forward plane crossings;
corner smoothing remains later work.
Physical corners, speed transitions and route timing still need validation on the robot.

The regional paths explicitly enable bounded intermediate crossing. AutoConstants sets provisional
limits of 0.25 m past the arrival plane and 0.5 m travel between position samples. The crossing and
current pose must also be inside the waypoint's lateral tolerance. Each section seeds its first
incoming leg from measured starting pose, processes multiple crossings in observed order, and
retains controller history when it changes targets. Rejected crossings do not restart deadlines.
Large sample jumps cannot prove crossing, while ordinary radius arrival still trusts current pose;
small vision corrections remain indistinguishable from movement. Braking bounds account for the
expanded handoff region. Crossing never completes an endpoint or bypasses an action boundary.
Validate the crossing bounds with real autonomous speeds, loop timing and vision corrections.

Regional driving sections also opt into CrossTrackController, using provisional AutoConstants
settings of 1.5/s lateral P gain and a 0.75 m/s correction cap. The active segment remains anchored
to its measured starting pose or preceding waypoint as the robot drifts and hands off targets.
Within that segment, lateral correction returns toward its line and forward motion eases near
the arrival plane. The combined translation request stays inside the existing braking speed
budget before passing through ChassisMotionLimiter. Outside the segment or on zero-length legs,
steering recovers directly toward the waypoint. Single-point commands and final aiming keep
correction disabled by default. Endpoint settling, deadlines and immediate fault stops still apply.
Validate the gain, correction cap, corner recovery and auto timing on hardware.

Build and simulate with the WPILib 2026 JDK:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2026/jdk'
./gradlew.bat build
./gradlew.bat simulateJava
```

Power Tool dependencies are installed locally and reproducible from `package-lock.json`.
On another checkout, run `npm ci` and `npm run build` from `power-tool`, then use
`power-tool/scripts/power-tool.cmd` to launch Electron. Build artifacts, dependencies,
and machine-specific paths are ignored.
