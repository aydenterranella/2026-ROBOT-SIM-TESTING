# AdvantageScope robot model

`Robot_9410_2026/` is a custom AdvantageScope robot built from the Onshape CAD, with the intake
arm and shooter arm as moving parts driven by the simulation.

| File | What it is |
|---|---|
| `config.json` | Frame conversion + component setup (checked in) |
| `model.glb` | Chassis and everything fixed to it (generated, not checked in — ~80 MB) |
| `model_0.glb` | Intake arm — component 0 (generated) |
| `model_1.glb` | Shooter/hood arm — component 1 (generated) |

## Why the config looks like that

The GLB is exported straight from the CAD Main Assembly (meters, Z up). The CAD frame is rotated
90° from the robot frame and its centerline is at CAD X = 17.047", so:

- `rotations: z +90°` turns CAD (X, Y, Z) into (−Y, X, Z) — CAD −Y is the robot's front.
- `position: [0, -0.43299, 0]` shifts the CAD centerline (0.43299 m) onto the robot center.

Both components use the **same** transform as the base, so they appear exactly where they sit in
CAD. The robot code then publishes poses on `Sim/Mechanisms/ComponentPoses` that rotate each
part about its pivot by (simulated angle − CAD angle). See `MechanismSimManager.rotateAboutPivot`.

## Regenerating the models (after a CAD change)

1. Onshape → **Main Assembly** → Export → Format **GLB**, "Y axis up" **unchecked**,
   Resolution Medium, Compress off.
2. From the repo root:
   ```
   node tools/advantagescope/split-robot-glb.mjs "<path to Main Assembly.glb>" advantagescope/Robot_9410_2026
   ```
3. Copy `advantagescope/Robot_9410_2026` into AdvantageScope's assets folder
   (AdvantageScope → Help → Show Assets Folder) and restart AdvantageScope.

If a new part shows up on the wrong component, add its Onshape instance ID to the fixed-part
lists at the top of `split-robot-glb.mjs`.

The script also fixes two export problems:
- **Transparency** — Onshape's GLB export makes every part opaque. Thin, large sheets in the
  chassis/intake get a see-through polycarbonate material (`POLYCARB` settings in the script).
- **CAD-only geometry** — the Limelight field-of-view cone is dropped (`SKIP_IDS`).

## Viewing it

1. Start the sim, open AdvantageScope, connect to the sim (File → Connect to Simulator).
2. Add a **3D Field** tab. Under Poses, drag `Robot/Pose` in as **Robot** and choose
   "9410 2026 Robot" as the model.
3. Drag `Sim/Mechanisms/ComponentPoses` onto the robot entry as **Component** poses.
4. Optional: drag `MapleSim/Fuel3d` in as **Game Piece** to see the fuel.
