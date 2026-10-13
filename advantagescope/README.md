# AdvantageScope robot model

`Robot_9410_2026/` is a custom AdvantageScope robot built from the Onshape CAD, with the intake
arm, shooter hood, hopper extension, and swerve wheels as moving parts driven by the simulation.

| File | What it is |
|---|---|
| `config.json` | Frame conversion + component setup (checked in, written by the split script) |
| `model.glb` | Chassis and everything fixed to it (generated, not checked in — ~85 MB) |
| `model_0.glb` | Intake arm — component 0 (generated) |
| `model_1.glb` | Shooter hood (inner links, gear sectors, upper rollers) — component 1 (generated) |
| `model_2.glb` | Hopper extension — component 2 (generated) |
| `model_3.glb` … `model_6.glb` | Swerve wheels FL, FR, BL, BR — components 3-6 (generated) |

## Why the config looks like that

The GLB is exported straight from the CAD Main Assembly (meters, Z up). The CAD frame is rotated
90° from the robot frame and its centerline is at CAD X = 17.047", so:

- `rotations: z +90°` turns CAD (X, Y, Z) into (−Y, X, Z) — CAD −Y is the robot's front.
- `position: [0, -0.43299, 0]` shifts the CAD centerline (0.43299 m) onto the robot center.

Every component uses the **same** transform as the base, so it appears exactly where it sits in
CAD. The robot code then publishes poses on `Sim/Mechanisms/ComponentPoses` that move each part
relative to its CAD pose (see `MechanismSimManager.publishTelemetry`):

- intake and hood rotate about their pivots by (simulated angle − CAD angle);
- the hopper extension slides back by however far it is from fully out (CAD draws it out);
- each wheel spins about its axle by the MapleSim wheel position, then steers from its CAD
  heading to the MapleSim module heading.

## Regenerating the models (after a CAD change)

1. Onshape → **Main Assembly** → Export → Format **GLB**, "Y axis up" **unchecked**,
   Resolution Medium, Compress off.
2. From the repo root:
   ```
   node tools/advantagescope/split-robot-glb.mjs "<path to Main Assembly.glb>" advantagescope/Robot_9410_2026
   ```
3. Copy `advantagescope/Robot_9410_2026` into AdvantageScope's assets folder
   (AdvantageScope → Help → Show Assets Folder) and restart AdvantageScope.
4. The script prints each swerve wheel's center and CAD heading. If they changed, update
   `WHEEL_CENTERS` / `WHEEL_CAD_STEER_RAD` in `MechanismSimConstants`.

If a part shows up on the wrong component, add its Onshape instance ID to the lists at the top of
`split-robot-glb.mjs` (`INTAKE_FIXED_IDS`, `HOOD_IDS`, `HOPPER_IDS`, `SKIP_IDS`).

The script also fixes some export problems:
- **Transparency** — Onshape's GLB export makes every part opaque. Thin, large sheets in the
  chassis/intake get a see-through polycarbonate material (`POLYCARB` settings in the script).
- **CAD-only geometry** — the three Limelights' field-of-view pyramids, the two fuel balls
  sitting in the CAD, and two stray parts floating off the robot's side are dropped (`LIMELIGHT_IDS`, `SKIP_IDS`).
- **Merged swerve modules** — each WCP Swerve X2 exports as one mesh; the wheel is cut out of it
  by finding the tread and keeping every face inside the tread's cylinder.

## Viewing it

1. Start the sim, open AdvantageScope, connect to the sim (File → Connect to Simulator).
2. Add a **3D Field** tab. Under Poses, drag `MapleSim/RobotPose3d` in as **Robot** (it includes
   bump height and tilt; `Robot/Pose` also works but stays flat) and choose "9410 2026 Robot"
   as the model.
3. Drag `Sim/Mechanisms/ComponentPoses` onto the robot entry as **Component** poses.
4. Drag `MapleSim/Fuel3d` in as **Game Piece** to see the fuel. Fuel the robot is holding is
   drawn inside the hopper, one ball per piece, so you can see how full it is.
