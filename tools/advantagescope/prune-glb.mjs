// Removes nodes from a GLB by name + robot-frame bounding box, leaving all buffers untouched.
// usage: node prune-glb.mjs <in.glb> <out.glb|--dry-run> <rules.json>
import fs from 'node:fs';
const [inFile, outFile, rulesFile] = process.argv.slice(2);
const rules = JSON.parse(fs.readFileSync(rulesFile, 'utf8'));
const buf = fs.readFileSync(inFile);
const jsonLen = buf.readUInt32LE(12);
const json = JSON.parse(buf.subarray(20, 20 + jsonLen).toString());
const rest = buf.subarray(20 + jsonLen); // BIN chunk header + data, copied as-is
const IN = 0.0254, C = 0.43299, I = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
const mul = (a, b) => { const r = new Array(16).fill(0); for (let c = 0; c < 4; c++) for (let w = 0; w < 4; w++) for (let k = 0; k < 4; k++) r[c*4+w] += a[k*4+w] * b[c*4+k]; return r; };
const ap = (m, p) => [0,1,2].map(i => m[i]*p[0] + m[4+i]*p[1] + m[8+i]*p[2] + m[12+i]);
const rb = p => [-p[1]/IN, (p[0]-C)/IN, p[2]/IN];
const parentOf = new Map();
json.nodes.forEach((n, i) => (n.children ?? []).forEach(c => parentOf.set(c, i)));
const hits = new Map();
function walk(i, W, path) {
  const n = json.nodes[i]; const w = n.matrix ? mul(W, n.matrix) : W; const p = path.concat([n.name ?? '']);
  if (n.mesh !== undefined) {
    let lo = [1e9,1e9,1e9], hi = [-1e9,-1e9,-1e9];
    for (const pr of json.meshes[n.mesh].primitives) { const a = json.accessors[pr.attributes.POSITION];
      for (const x of [a.min[0], a.max[0]]) for (const y of [a.min[1], a.max[1]]) for (const z of [a.min[2], a.max[2]]) {
        const r = rb(ap(w, [x,y,z])); lo = lo.map((v,k) => Math.min(v, r[k])); hi = hi.map((v,k) => Math.max(v, r[k])); } }
    const full = p.filter(Boolean).join(' / ');
    for (const rule of rules) {
      if (!new RegExp(rule.name, 'i').test(full)) continue;
      if (rule.path && !new RegExp(rule.path).test(full)) continue;
      const b = rule.box; // inches, robot frame: node bbox must sit fully inside
      if (lo[0] < b.xMin || hi[0] > b.xMax || lo[1] < b.yMin || hi[1] > b.yMax || lo[2] < b.zMin || hi[2] > b.zMax) continue;
      hits.set(i, { rule: rule.label, full, lo, hi }); break;
    }
  }
  for (const c of n.children ?? []) walk(c, w, p);
}
for (const r of json.scenes[0].nodes) walk(r, I, []);
const counts = {};
for (const [i, h] of hits) { counts[h.rule] = (counts[h.rule] ?? 0) + 1;
  console.log(`${h.rule.padEnd(14)} node ${String(i).padStart(4)}  ${h.full.replace(/occurrence of /g, '').slice(0, 95)}  [${h.lo.map(v => v.toFixed(2))}]..[${h.hi.map(v => v.toFixed(2))}]`); }
console.log('counts', JSON.stringify(counts), 'total', hits.size);
if (outFile === '--dry-run') process.exit(0);
for (const i of hits.keys()) {
  const p = parentOf.get(i);
  if (p !== undefined) json.nodes[p].children = json.nodes[p].children.filter(c => c !== i);
  json.scenes.forEach(s => { s.nodes = s.nodes.filter(c => c !== i); });
}
let text = JSON.stringify(json); while (Buffer.byteLength(text) % 4) text += ' ';
const jsonBuf = Buffer.from(text);
const header = Buffer.alloc(12); header.writeUInt32LE(0x46546C67, 0); header.writeUInt32LE(2, 4); header.writeUInt32LE(12 + 8 + jsonBuf.length + rest.length, 8);
const chunkHeader = Buffer.alloc(8); chunkHeader.writeUInt32LE(jsonBuf.length, 0); chunkHeader.writeUInt32LE(0x4E4F534A, 4);
fs.writeFileSync(outFile, Buffer.concat([header, chunkHeader, jsonBuf, rest]));
console.log('wrote', outFile);
