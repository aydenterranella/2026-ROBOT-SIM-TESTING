// Splits the full-robot GLB exported from Onshape into the separate models AdvantageScope needs:
//
//   model.glb    everything that doesn't move relative to the chassis
//   model_0.glb  intake arm   (component 0 — rotates about the intake pivot)
//   model_1.glb  shooter arm  (component 1 — rotates about the flywheel axle)
//
// The component order must match the Pose3d[] published on "Sim/Mechanisms/ComponentPoses" by
// frc.robot.sim.MechanismSimManager.
//
// HOW TO RE-RUN AFTER A CAD CHANGE
//   1. In Onshape, open "Main Assembly" → Export → Format: GLB, leave "Y axis up" UNCHECKED,
//      Resolution: Medium, Compress: OFF.
//   2. node tools/advantagescope/split-robot-glb.mjs "<path to Main Assembly.glb>" advantagescope/Robot_9410_2026
//   3. Copy the output folder into AdvantageScope's assets folder
//      (AdvantageScope → Help → Show Assets Folder), then restart AdvantageScope.
//
// No npm packages needed — this only reads/writes the GLB container format directly.
//
// HOW PARTS ARE SORTED
// Onshape's export loses instance numbers in part names ("Part 1" appears dozens of times), so we
// sort parts by WHERE they are (bounding box in robot coordinates) plus a short list of
// exceptions keyed by Onshape instance ID (stable across re-exports, unlike node order).

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

/** Shooter arm: everything in the "Shooter V2 <1>" subassembly EXCEPT these fixed parts. */
const SHOOTER_SUBASSEMBLY = 'Shooter V2 <1>';
const SHOOTER_FIXED_IDS = new Set([
  'MtvBH30jG1CqAuWgb', // baseplate
  'MCDxd2Y5QAUgpuCR0', 'MBYcm6NQ4KPoSLouP', // 190t gear sectors (the pinion climbs these)
  'McYZDbxKoLRGUFt9j', 'MvPz7e2rVjlej1x4g', 'MbjpD9G3eG0bb77BK', 'M05jDAAa+nlY/MU/T', // sector spacers
  'Mi4yAAunE2iVzqRum', 'M4af3OST5gOqElZ4g', 'M/XmFvvhwM95Lm8oO', // sector mounts
  'MYjOWT9FIHaJWEOLY', // hood WCP Through Bore Encoder
  'MTY3F0TodM0E5KPxr', 'M5V06ZMIWdCPD8dJX', // encoder bracket
  'MlvpjqPw0uRxB7Cqj', 'MpWkN/kmcP6auBSTu', // encoder shaft + 24t gear
  'MeeryRzsqRsNm9Oca', 'MtMIFDZfgsAvZQoo8', 'MI3kxNLs7lB/kN1gr', // encoder shaft bushings/spacer
]);

/** CAD-only helper geometry that shouldn't be drawn at all. */
const SKIP_IDS = new Set([
  'MCjYC3L7/8IVFhuXs', // Limelight field-of-view cone (the gray pyramid off the back)
]);

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

// ---------------------------------------------------------------------------------------------
// Classify every mesh node as 'base', 'intake', or 'shooter'
// ---------------------------------------------------------------------------------------------
function classify(json) {
  const result = new Map(); // mesh node index -> category
  const polycarbMeshes = new Set(); // mesh indices to recolor as clear polycarbonate
  const walk = (index, parentWorld, subassembly, instanceId) => {
    const node = json.nodes[index];
    const world = node.matrix ? mul(parentWorld, node.matrix) : parentWorld;
    const meta = node.extensions?.PTC_onshape_metadata;
    const id = meta?.id ? meta.id[meta.id.length - 1] : instanceId;
    // Remember the top-level subassembly ("Main <1>", "Shooter V2 <1>") we're inside of.
    const isSubassembly = node.mesh === undefined && !String(node.name).startsWith('occurrence of');
    const sub = subassembly ?? (index !== json.scenes[0].nodes[0] && isSubassembly ? node.name : null);

    if (node.mesh !== undefined) {
      let category = 'base';
      if (sub === 'Main <1>') {
        const { lo, hi } = boundsInches(json, node.mesh, world);
        const dims = [0, 1, 2].map((k) => hi[k] - lo[k]).sort((a, b) => a - b);
        if (dims[0] <= POLYCARB.maxThicknessIn && dims[2] >= POLYCARB.minSpanIn) polycarbMeshes.add(node.mesh);
      }
      if (SKIP_IDS.has(id)) {
        category = 'skip';
      } else if (sub === SHOOTER_SUBASSEMBLY) {
        category = SHOOTER_FIXED_IDS.has(id) ? 'base' : 'shooter';
      } else if ((sub === 'Main <1>' || sub === null) && !INTAKE_FIXED_IDS.has(id)) {
        // sub === null: parts inserted directly into the top-level Main Assembly (a few intake
        // roller pulleys live there instead of inside "Main <1>").
        const { lo, hi } = boundsInches(json, node.mesh, world);
        const b = INTAKE_BOX_IN;
        if (lo[0] >= b.xMin && hi[0] <= b.xMax && lo[2] >= b.zMin && hi[2] <= b.zMax) category = 'intake';
      }
      result.set(index, category);
    }
    for (const child of node.children ?? []) walk(child, world, sub, id);
  };
  for (const root of json.scenes[0].nodes) walk(root, IDENTITY, null, null);
  return { categories: result, polycarbMeshes };
}

/** Points every primitive of the given meshes at a new see-through material. */
function applyPolycarb(json, meshIndices) {
  const materialIndex = json.materials.length;
  json.materials.push(POLYCARB_MATERIAL);
  for (const m of meshIndices) for (const prim of json.meshes[m].primitives) prim.material = materialIndex;
}

// ---------------------------------------------------------------------------------------------
// Build a pruned copy of the GLB containing only the mesh nodes in `keep`
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
  const mapMesh = (m) => {
    if (!meshMap.has(m)) {
      const mesh = json.meshes[m];
      meshMap.set(m, newMeshes.length);
      newMeshes.push({
        ...mesh,
        primitives: mesh.primitives.map((p) => ({
          ...p,
          attributes: Object.fromEntries(Object.entries(p.attributes).map(([k, a]) => [k, mapAccessor(a)])),
          ...(p.indices !== undefined ? { indices: mapAccessor(p.indices) } : {}),
        })),
      });
    }
    return meshMap.get(m);
  };

  out.nodes = [...keepNode.entries()].map(([old, kids]) => {
    const n = { ...json.nodes[old], children: kids.map((k) => nodeMap.get(k)) };
    if (n.children.length === 0) delete n.children;
    if (n.mesh !== undefined) {
      if (keep.has(old)) n.mesh = mapMesh(n.mesh);
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
const { categories, polycarbMeshes } = classify(json);
applyPolycarb(json, polycarbMeshes);
console.log(`polycarbonate sheets: ${polycarbMeshes.size} meshes made see-through`);
fs.mkdirSync(outDir, { recursive: true });

const outputs = { base: 'model.glb', intake: 'model_0.glb', shooter: 'model_1.glb' };
for (const [category, file] of Object.entries(outputs)) {
  const keep = new Set([...categories].filter(([, c]) => c === category).map(([i]) => i));
  const part = extract(json, bin, keep);
  writeGlb(path.join(outDir, file), part.json, part.bin);
  console.log(`${file.padEnd(12)} ${String(keep.size).padStart(4)} parts  ${(part.bin.length / 1e6).toFixed(1)} MB`);
}
