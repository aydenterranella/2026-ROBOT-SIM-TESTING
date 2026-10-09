// Splits the full-robot GLB exported from Onshape into the separate models AdvantageScope needs:
//
//   model.glb    everything that doesn't move relative to the chassis
//   model_0.glb  intake arm        (component 0 — rotates about the intake pivot)
//   model_1.glb  shooter hood      (component 1 — rotates about the flywheel axle)
//   model_2.glb  hopper extension  (component 2 — slides forward along +X)
//   model_3.glb  FL swerve wheel   (components 3-6 — steer about the module's vertical axis and
//   model_4.glb  FR swerve wheel    spin about the axle)
//   model_5.glb  BL swerve wheel
//   model_6.glb  BR swerve wheel
//
// It also writes config.json next to the models. The component order must match the Pose3d[]
// published on "Sim/Mechanisms/ComponentPoses" by frc.robot.sim.MechanismSimManager.
//
// HOW TO RE-RUN AFTER A CAD CHANGE
//   1. In Onshape, open "Main Assembly" → Export → Format: GLB, leave "Y axis up" UNCHECKED,
//      Resolution: Medium, Compress: OFF.
//   2. node tools/advantagescope/split-robot-glb.mjs "<path to Main Assembly.glb>" advantagescope/Robot_9410_2026
//   3. Copy the output folder into AdvantageScope's assets folder
//      (AdvantageScope → Help → Show Assets Folder), then restart AdvantageScope.
//   4. If the script prints different swerve wheel numbers than WHEEL_* in
//      src/main/java/frc/robot/sim/MechanismSimConstants.java, update those constants.
//
// No npm packages needed — this only reads/writes the GLB container format directly.
//
// HOW PARTS ARE SORTED
// Onshape's export loses instance numbers in part names ("Part 1" appears dozens of times), so we
// sort parts by WHERE they are (bounding box in robot coordinates) plus lists keyed by Onshape
// instance ID (stable across re-exports, unlike node order).

import fs from 'node:fs';
import path from 'node:path';

// ---------------------------------------------------------------------------------------------
// Frame conversion. CAD Main Assembly (meters, Z up) → WPILib robot frame:
//   x_robot = -Y_cad,  y_robot = X_cad - CAD_CENTERLINE_X,  z_robot = Z_cad
// Matches the conversion documented in src/main/java/frc/robot/sim/MechanismSimConstants.java.
// ---------------------------------------------------------------------------------------------
const CAD_CENTERLINE_X_M = 0.43299; // 17.047 in
const IN = 0.0254;

/** Intake arm: loose parts in "Main <1>" (or the top level) inside this box (robot frame, inches). */
const INTAKE_BOX_IN = { xMin: 10.5, xMax: 27.5, zMin: 0.5, zMax: 14.0 };

/** Parts inside the intake box that are bolted to the chassis, not the arm. */
const INTAKE_FIXED_IDS = new Set([
  'Mx1qgqh9zoXdYRn3i', // WCP Through Bore Encoder (pivot encoder body)
  'MbjxQ8PnqIKovzd+t', 'MMqvYtdfa9bCjUejE', // nut strips on the pivot mount plates
  'Maia4BxhQwJuavXeu', 'MUxwTUvMRHgpwrSs8', // pivot gearbox parts
  'MQ0ZnPfJXq7F+quaj', 'MdXOM+a2MTnzSbkrJ', // "Right"/"Left" gearbox plates
  'MQX7twKjbZQ0BW1bh', 'MCePttoE8eZUIpBdf', // brackets below the pivot
  'Mygtt5NQPJpPEbPG9', 'MDd3Yr60c3NpNMk4D', // nut strips above the pivot
  'MX+XndSBa7JUq3+Gd', // 1/2" standoff
  'MKUqRI9CgnlCjaz4q', // 2x1 front frame tube
  'MHsFhexU05WLkrFaE', // 3/8" hex standoff
  'MudGaSqVMsSldz4mb', 'Izgmr5S+taQ0/vvsB', // low brackets
  'MgiexC97tB5MW6Zeo', // screw
]);

/**
 * Shooter hood: the inner body of "Shooter V2". Two narrow truss links ride on the flywheel axle
 * and carry the two upper roller shafts plus the 190t gear sectors (tooth tips sit 9.6" from the
 * flywheel axle). The hood motor and its 10t pinion are on the FIXED outer side plates, so the
 * pinion drives the sectors and the whole inner body swings about the flywheel axle.
 */
const HOOD_IDS = new Set([
  'M6bh6uQHQrlT+PJA3', 'Man9zclAzdkvAVS9+', // narrow truss links ("Part 3")
  'MBYcm6NQ4KPoSLouP', 'MCDxd2Y5QAUgpuCR0', // 190t gear sectors
  'Mi4yAAunE2iVzqRum', 'M4af3OST5gOqElZ4g', // spacer plates between the sectors
  'McYZDbxKoLRGUFt9j', 'MvPz7e2rVjlej1x4g', 'M05jDAAa+nlY/MU/T', // sector standoffs
  'Mti3MzRAFHA8fRjj/', 'MbjpD9G3eG0bb77BK', // upper roller shaft + roller
  'MIKM80QBr+ac7H+uO', // lower roller shaft
  'Mj+0aGc7VUvDkO0iA', 'ME0gJXnmStPCyfNkv', // 20t / 38t gears between the two roller shafts
  'M6hpKt0vft1r3CAZv', 'MGlmzt77BMFGQhqIN', // 24t pulleys on the lower roller shaft
  'MplDGshjBfUuy5x/w', 'MxBNzwG+w8Nzec3fE', // 110T belts (flywheel axle → lower roller)
  'MoYLJFyKNmkw6rgRR', 'IKFthNu+plzBeNr5p', 'IBeNFX2D2i4qjvSQr', 'IuBC1WG2Qi7mMxaha', // bearings
  'I/ge4T5lKy8t1LMfl', 'IBu57JRIDpxCGFYgM', 'ICkB9aMiNMOmCkppM', 'I23aJ55tTctgL0rQP',
  'Mf+sKHTibbgHtYMd8', 'M5nKMiL0ohc1qh0OE', 'MHshUXjS7KP6PHNzR', 'M2qMcVd0PvZBtF59j', // spacers
  'MzChha1YMssUZxyfK', 'Myum3arAUQvIPTpRH', 'Mj8+KDbjJirn8Umzk', 'M7fHLkhpNaH1c1//i',
  'MGru7VseeVNCnr7ZR', 'MdhkmQ09zUXSECJO2',
  'IWjE6Fr6hqjXmk48x', 'Ibi77cHMisU7hNsk6', 'I+v4ELXYIsV81UK05', 'I/OFN9/3o03DvZGZ5', // link screws
  'Ir8aTR/LYhJcAEFdH', 'I7FC2SKg9nBMKXugY', 'Iz4/a64fQAD7hWKVH', 'IiQAXeRzECY9Cj3pC',
  'MyVX0UsBznkJXeUJ3', 'MHQnuD4K1zSyqDwBZ', // shaft end screws
]);
const SHOOTER_SUBASSEMBLY = 'Shooter V2 <1>';

/**
 * Hopper extension: the outer box (two side walls, front wall, nut strips) that slides forward
 * over the deployed intake. The CAD draws it fully extended.
 */
const HOPPER_IDS = new Set([
  'MpZULS2ip9qMB6CZN', 'MRYkGVmbQtf6JsqUl', // side walls
  'MRTqL43/k29tPqyiK', // front wall
  'MYMOPR2+St65j6zuU', 'MXYmDW0Oxs7IKFDdv', 'MPvwlpcPggFPIeXsq', 'MebIBjurmrVzY3fMm', // nut strips
]);

/** Parts that shouldn't be drawn at all. */
const SKIP_IDS = new Set([
  'MArO9ax+VvjWWOR/K', 'MTnyU2kiTHgO3Gjrv', // fuel balls sitting in the CAD (sim draws real ones)
  'MFmHKN/FWRjyAGhsc', // stray part floating ~21" off the robot's right side, next to the intake
  'M9a9+YSicYzUgKYgb', // stray 1.5" spacer floating ~16.5" off the robot's right side
]);

/**
 * Limelight subassemblies. Each one carries a field-of-view pyramid: an unnamed mesh that shares
 * the Limelight's instance ID. Only that unnamed mesh is skipped; the camera body stays.
 */
const LIMELIGHT_IDS = new Set(['MCjYC3L7/8IVFhuXs', 'MIdNYZeD+J8lJANF5', 'MDBHNxrH0Zpx8JGAI']);

/** Swerve modules are one merged mesh each; the wheel is cut out of it by geometry. */
const SWERVE_SUBASSEMBLY_PREFIX = 'WCP Swerve X2';
/** The wheel tread's material (light blue-gray) — used to find the wheel inside each module. */
const TREAD_MATERIAL_PREFIX = '0.811765_0.858824_0.898039';

/**
 * Onshape's GLB export drops part transparency, so the polycarbonate panels come out solid gray.
 * Any thin, large sheet in the chassis/intake ("Main <1>") gets this see-through material instead.
 * (The shooter's side plates are aluminum and use the same gray, so the shooter is excluded.)
 */
const POLYCARB = { maxThicknessIn: 0.3, minSpanIn: 12 };
const POLYCARB_MATERIAL = {
  name: 'polycarbonate (added by split-robot-glb)',
  doubleSided: true,
  alphaMode: 'BLEND',
  pbrMetallicRoughness: { baseColorFactor: [0.82, 0.88, 0.95, 0.3], metallicFactor: 0, roughnessFactor: 0.2 },
};

/** AdvantageScope transform that puts the CAD frame into the robot frame (see top of file). */
const ZEROED = {
  rotations: [{ axis: 'z', degrees: 90 }],
  position: [0, -CAD_CENTERLINE_X_M, 0],
};

// ---------------------------------------------------------------------------------------------
// GLB reading/writing
// ---------------------------------------------------------------------------------------------
function readGlb(file) {
  const buf = fs.readFileSync(file);
  if (buf.readUInt32LE(0) !== 0x46546c67) throw new Error('Not a GLB file');
  const jsonLen = buf.readUInt32LE(12);
  const json = JSON.parse(buf.subarray(20, 20 + jsonLen).toString('utf8'));
  const binStart = 20 + jsonLen;
  const binLen = buf.readUInt32LE(binStart);
  const bin = buf.subarray(binStart + 8, binStart + 8 + binLen);
  return { json, bin };
}

function writeGlb(file, json, bin) {
  const pad = (b, fill) => (b.length % 4 ? Buffer.concat([b, Buffer.alloc(4 - (b.length % 4), fill)]) : b);
  const jsonBuf = pad(Buffer.from(JSON.stringify(json), 'utf8'), 0x20);
  const binBuf = pad(bin, 0);
  const header = Buffer.alloc(12);
  header.writeUInt32LE(0x46546c67, 0);
  header.writeUInt32LE(2, 4);
  header.writeUInt32LE(12 + 8 + jsonBuf.length + 8 + binBuf.length, 8);
  const chunk = (len, type) => {
    const h = Buffer.alloc(8);
    h.writeUInt32LE(len, 0);
    h.writeUInt32LE(type, 4);
    return h;
  };
  fs.writeFileSync(
    file,
    Buffer.concat([header, chunk(jsonBuf.length, 0x4e4f534a), jsonBuf, chunk(binBuf.length, 0x004e4942), binBuf]),
  );
}

// ---------------------------------------------------------------------------------------------
// Geometry helpers (glTF matrices are column-major)
// ---------------------------------------------------------------------------------------------
const IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
function mul(a, b) {
  const r = new Array(16).fill(0);
  for (let col = 0; col < 4; col++)
    for (let row = 0; row < 4; row++)
      for (let k = 0; k < 4; k++) r[col * 4 + row] += a[k * 4 + row] * b[col * 4 + k];
  return r;
}
const apply = (m, p) => [0, 1, 2].map((i) => m[i] * p[0] + m[4 + i] * p[1] + m[8 + i] * p[2] + m[12 + i]);
const cadToRobotInches = (p) => [-p[1] / IN, (p[0] - CAD_CENTERLINE_X_M) / IN, p[2] / IN];

/** World-space bounding box of a mesh node, in robot-frame inches. */
function boundsInches(json, meshIndex, world) {
  let lo = [Infinity, Infinity, Infinity];
  let hi = [-Infinity, -Infinity, -Infinity];
  for (const prim of json.meshes[meshIndex].primitives) {
    const acc = json.accessors[prim.attributes.POSITION];
    for (const x of [acc.min[0], acc.max[0]])
      for (const y of [acc.min[1], acc.max[1]])
        for (const z of [acc.min[2], acc.max[2]]) {
          const r = cadToRobotInches(apply(world, [x, y, z]));
          lo = lo.map((v, k) => Math.min(v, r[k]));
          hi = hi.map((v, k) => Math.max(v, r[k]));
        }
  }
  return { lo, hi };
}

/** Every vertex of one primitive, in robot-frame inches. */
function primVertsInches(json, bin, prim, world) {
  const acc = json.accessors[prim.attributes.POSITION];
  const view = json.bufferViews[acc.bufferView];
  const offset = (view.byteOffset ?? 0) + (acc.byteOffset ?? 0);
  const stride = view.byteStride ?? 12;
  const out = [];
  for (let i = 0; i < acc.count; i++) {
    const o = offset + i * stride;
    out.push(cadToRobotInches(apply(world, [bin.readFloatLE(o), bin.readFloatLE(o + 4), bin.readFloatLE(o + 8)])));
  }
  return out;
}

/**
 * Finds the wheel inside one merged swerve-module mesh. Returns the primitive indices that belong
 * to the wheel plus the wheel's center and axle direction (robot frame).
 */
function findWheel(json, bin, meshIndex, world) {
  const prims = json.meshes[meshIndex].primitives;
  const tread = prims
    .map((p, k) => k)
    .filter((k) => json.materials[prims[k].material]?.name?.startsWith(TREAD_MATERIAL_PREFIX));
  if (tread.length === 0) throw new Error('Swerve module has no tread material; update TREAD_MATERIAL_PREFIX');
  const treadVerts = tread.flatMap((k) => primVertsInches(json, bin, prims[k], world));

  const lo = [0, 1, 2].map((j) => Math.min(...treadVerts.map((v) => v[j])));
  const hi = [0, 1, 2].map((j) => Math.max(...treadVerts.map((v) => v[j])));
  const center = [0, 1, 2].map((j) => (lo[j] + hi[j]) / 2);

  // The axle is horizontal: it's the minor axis of the tread's footprint in XY.
  let sxx = 0, syy = 0, sxy = 0;
  for (const v of treadVerts) {
    const dx = v[0] - center[0], dy = v[1] - center[1];
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  const majorAngle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
  const axle = [-Math.sin(majorAngle), Math.cos(majorAngle), 0];

  const axial = (v) => (v[0] - center[0]) * axle[0] + (v[1] - center[1]) * axle[1];
  const radial = (v) => {
    const a = axial(v);
    return Math.hypot(v[0] - center[0] - a * axle[0], v[1] - center[1] - a * axle[1], v[2] - center[2]);
  };
  const halfWidth = Math.max(...treadVerts.map((v) => Math.abs(axial(v))));
  const radius = Math.max(...treadVerts.map(radial));

  // A primitive is part of the wheel if all of it sits inside the tread's cylinder.
  const wheelPrims = new Set();
  prims.forEach((p, k) => {
    const verts = primVertsInches(json, bin, p, world);
    if (verts.every((v) => Math.abs(axial(v)) <= halfWidth * 1.2 + 0.05 && radial(v) <= radius + 0.05)) {
      wheelPrims.add(k);
    }
  });
  return { wheelPrims, center, axle, radius };
}

// ---------------------------------------------------------------------------------------------
// Classify every mesh node. Result: node index -> category, or node index -> Map(category -> prim
// indices) for swerve modules, which are split between the chassis and a wheel.
// ---------------------------------------------------------------------------------------------
function classify(json, bin) {
  const result = new Map();
  const polycarbMeshes = new Set(); // mesh indices to recolor as clear polycarbonate
  const wheels = []; // { node, wheelPrims, center, axle, radius }
  const walk = (index, parentWorld, subassembly, instanceId, inSwerve) => {
    const node = json.nodes[index];
    const world = node.matrix ? mul(parentWorld, node.matrix) : parentWorld;
    const meta = node.extensions?.PTC_onshape_metadata;
    const id = meta?.id ? meta.id[meta.id.length - 1] : instanceId;
    // Remember the top-level subassembly ("Main <1>", "Shooter V2 <1>") we're inside of.
    const isSubassembly = node.mesh === undefined && !String(node.name).startsWith('occurrence of');
    const sub = subassembly ?? (index !== json.scenes[0].nodes[0] && isSubassembly ? node.name : null);
    const swerve = inSwerve || (isSubassembly && String(node.name).startsWith(SWERVE_SUBASSEMBLY_PREFIX));

    if (node.mesh !== undefined) {
      let category = 'base';
      if (sub === 'Main <1>') {
        const { lo, hi } = boundsInches(json, node.mesh, world);
        const dims = [0, 1, 2].map((k) => hi[k] - lo[k]).sort((a, b) => a - b);
        if (dims[0] <= POLYCARB.maxThicknessIn && dims[2] >= POLYCARB.minSpanIn) polycarbMeshes.add(node.mesh);
      }
      if (SKIP_IDS.has(id) || (LIMELIGHT_IDS.has(id) && !node.name)) {
        category = 'skip';
      } else if (swerve) {
        wheels.push({ node: index, ...findWheel(json, bin, node.mesh, world) });
        category = null; // filled in below once the wheels are sorted FL/FR/BL/BR
      } else if (HOPPER_IDS.has(id)) {
        category = 'hopper';
      } else if (sub === SHOOTER_SUBASSEMBLY) {
        category = HOOD_IDS.has(id) ? 'hood' : 'base';
      } else if ((sub === 'Main <1>' || sub === null) && !INTAKE_FIXED_IDS.has(id)) {
        // sub === null: parts inserted directly into the top-level Main Assembly (a few intake
        // roller pulleys live there instead of inside "Main <1>").
        const { lo, hi } = boundsInches(json, node.mesh, world);
        const b = INTAKE_BOX_IN;
        if (lo[0] >= b.xMin && hi[0] <= b.xMax && lo[2] >= b.zMin && hi[2] <= b.zMax) category = 'intake';
      }
      if (category) result.set(index, category);
    }
    for (const child of node.children ?? []) walk(child, world, sub, id, swerve);
  };
  for (const root of json.scenes[0].nodes) walk(root, IDENTITY, null, null, false);

  // Order the wheels FL, FR, BL, BR (front = +X, left = +Y), matching the CTRE module order.
  if (wheels.length !== 4) throw new Error(`expected 4 swerve modules, found ${wheels.length}`);
  const order = (w) => (w.center[0] > 0 ? 0 : 2) + (w.center[1] > 0 ? 0 : 1);
  wheels.sort((a, b) => order(a) - order(b));
  wheels.forEach((w, i) => {
    const allPrims = json.meshes[json.nodes[w.node].mesh].primitives.map((p, k) => k);
    result.set(
      w.node,
      new Map([
        ['base', new Set(allPrims.filter((k) => !w.wheelPrims.has(k)))],
        [`wheel${i}`, w.wheelPrims],
      ]),
    );
  });
  return { categories: result, polycarbMeshes, wheels };
}

/** Points every primitive of the given meshes at a new see-through material. */
function applyPolycarb(json, meshIndices) {
  const materialIndex = json.materials.length;
  json.materials.push(POLYCARB_MATERIAL);
  for (const m of meshIndices) for (const prim of json.meshes[m].primitives) prim.material = materialIndex;
}

// ---------------------------------------------------------------------------------------------
// Build a pruned copy of the GLB. `keep` maps mesh node index -> null (whole mesh) or a Set of the
// primitive indices to keep.
// ---------------------------------------------------------------------------------------------
function extract(json, bin, keep) {
  const out = { ...json };
  // 1. Nodes: keep a node if it is a kept mesh node or has a kept descendant.
  const keepNode = new Map();
  const visit = (i) => {
    const n = json.nodes[i];
    const kids = (n.children ?? []).filter(visit);
    const keepIt = keep.has(i) || kids.length > 0;
    if (keepIt) keepNode.set(i, kids);
    return keepIt;
  };
  json.scenes[0].nodes.forEach(visit);
  const nodeMap = new Map([...keepNode.keys()].map((old, i) => [old, i]));

  // 2. Meshes / accessors / bufferViews actually referenced.
  const meshMap = new Map();
  const accMap = new Map();
  const viewMap = new Map();
  const newMeshes = [];
  const newAccessors = [];
  const newViews = [];
  const binParts = [];
  let binOffset = 0;
  const mapView = (v) => {
    if (!viewMap.has(v)) {
      const view = json.bufferViews[v];
      const bytes = bin.subarray(view.byteOffset ?? 0, (view.byteOffset ?? 0) + view.byteLength);
      const padding = (4 - (binOffset % 4)) % 4;
      if (padding) {
        binParts.push(Buffer.alloc(padding));
        binOffset += padding;
      }
      viewMap.set(v, newViews.length);
      newViews.push({ ...view, buffer: 0, byteOffset: binOffset });
      binParts.push(bytes);
      binOffset += bytes.length;
    }
    return viewMap.get(v);
  };
  const mapAccessor = (a) => {
    if (!accMap.has(a)) {
      const acc = { ...json.accessors[a] };
      if (acc.bufferView !== undefined) acc.bufferView = mapView(acc.bufferView);
      accMap.set(a, newAccessors.length);
      newAccessors.push(acc);
    }
    return accMap.get(a);
  };
  const mapMesh = (m, prims) => {
    const key = prims ? `${m}:${[...prims].sort((a, b) => a - b).join(',')}` : `${m}`;
    if (!meshMap.has(key)) {
      const mesh = json.meshes[m];
      meshMap.set(key, newMeshes.length);
      newMeshes.push({
        ...mesh,
        primitives: mesh.primitives
          .filter((p, k) => !prims || prims.has(k))
          .map((p) => ({
            ...p,
            attributes: Object.fromEntries(Object.entries(p.attributes).map(([k, a]) => [k, mapAccessor(a)])),
            ...(p.indices !== undefined ? { indices: mapAccessor(p.indices) } : {}),
          })),
      });
    }
    return meshMap.get(key);
  };

  out.nodes = [...keepNode.entries()].map(([old, kids]) => {
    const n = { ...json.nodes[old], children: kids.map((k) => nodeMap.get(k)) };
    if (n.children.length === 0) delete n.children;
    if (n.mesh !== undefined) {
      if (keep.has(old)) n.mesh = mapMesh(n.mesh, keep.get(old));
      else delete n.mesh;
    }
    return n;
  });
  out.scenes = [{ ...json.scenes[0], nodes: json.scenes[0].nodes.filter((r) => nodeMap.has(r)).map((r) => nodeMap.get(r)) }];
  out.scene = 0;
  out.meshes = newMeshes;
  out.accessors = newAccessors;
  out.bufferViews = newViews;
  const newBin = Buffer.concat(binParts);
  out.buffers = [{ byteLength: newBin.length }];
  return { json: out, bin: newBin };
}

// ---------------------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------------------
const [input, outDir] = process.argv.slice(2);
if (!input || !outDir) {
  console.error('usage: node split-robot-glb.mjs "<Main Assembly.glb>" <output folder>');
  process.exit(1);
}
const { json, bin } = readGlb(input);
const { categories, polycarbMeshes, wheels } = classify(json, bin);
applyPolycarb(json, polycarbMeshes);
console.log(`polycarbonate sheets: ${polycarbMeshes.size} meshes made see-through`);
fs.mkdirSync(outDir, { recursive: true });

const outputs = [
  ['base', 'model.glb'],
  ['intake', 'model_0.glb'],
  ['hood', 'model_1.glb'],
  ['hopper', 'model_2.glb'],
  ['wheel0', 'model_3.glb'],
  ['wheel1', 'model_4.glb'],
  ['wheel2', 'model_5.glb'],
  ['wheel3', 'model_6.glb'],
];
for (const [category, file] of outputs) {
  const keep = new Map();
  for (const [node, c] of categories) {
    if (c === category) keep.set(node, null);
    else if (c instanceof Map && c.has(category)) keep.set(node, c.get(category));
  }
  const part = extract(json, bin, keep);
  writeGlb(path.join(outDir, file), part.json, part.bin);
  console.log(`${file.padEnd(12)} ${String(keep.size).padStart(4)} parts  ${(part.bin.length / 1e6).toFixed(1)} MB`);
}

const config = {
  name: '9410 2026 Robot',
  isFTC: false,
  ...ZEROED,
  cameras: [],
  components: outputs.slice(1).map(() => ({ zeroedRotations: ZEROED.rotations, zeroedPosition: ZEROED.position })),
};
fs.writeFileSync(path.join(outDir, 'config.json'), JSON.stringify(config, null, 2) + '\n');

// Numbers MechanismSimConstants needs to steer/spin the wheel components.
console.log('\nSwerve wheels (robot frame) — keep WHEEL_* in MechanismSimConstants in sync:');
['FL', 'FR', 'BL', 'BR'].forEach((name, i) => {
  const w = wheels[i];
  // Rolling direction = axle × up; report it as the CAD steer angle.
  const steerDeg = (Math.atan2(-w.axle[0], w.axle[1]) * 180) / Math.PI;
  console.log(
    `  ${name}: center (${w.center.map((v) => (v * IN).toFixed(4)).join(', ')}) m, ` +
      `CAD steer ${steerDeg.toFixed(1)} deg, radius ${(w.radius * IN).toFixed(4)} m, ${w.wheelPrims.size} prims`,
  );
});
