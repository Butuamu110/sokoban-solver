// Sokoban solver: A* in push space with tunnel macros and deterministic
// push-ordering heuristics. Plain JavaScript in a .ts file (Node 22 type
// stripping). No clock reads, no randomness: search budgets are node counts.
import { readdirSync, readFileSync, writeFileSync, mkdirSync, existsSync, statSync } from "node:fs";
import { join } from "node:path";

const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
];

// ---------- level parsing ----------
function parseLevel(text) {
  const rows = text.split(/\r?\n/);
  while (rows.length && rows[rows.length - 1] === "") rows.pop();
  const H = rows.length;
  let W = 0;
  for (const r of rows) W = Math.max(W, r.length);
  const N = W * H;
  const wall = new Uint8Array(N).fill(1);
  const goal = new Uint8Array(N);
  const boxes = [];
  let player = -1;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const c = x < rows[y].length ? rows[y][x] : "#";
      const i = y * W + x;
      if (c === "#") { wall[i] = 1; continue; }
      wall[i] = 0;
      if (c === "." || c === "*" || c === "+") goal[i] = 1;
      if (c === "$" || c === "*") boxes.push(i);
      if (c === "@" || c === "+") player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { W, H, N, wall, goal, boxes, player };
}

// ---------- static push-distance table (reverse pull BFS, min over goals) ----------
function computeDist(L) {
  const { W, N, wall, goal } = L;
  const dist = new Int32Array(N).fill(-1);
  const q = [];
  for (let i = 0; i < N; i++) if (goal[i] && !wall[i]) { dist[i] = 0; q.push(i); }
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    const cx = c % W, cy = (c / W) | 0;
    for (let d = 0; d < 4; d++) {
      // box moved from p to c along d, so p = c - d, stance = p - d
      const px = cx - DIRS[d].dx, py = cy - DIRS[d].dy;
      const sx = px - DIRS[d].dx, sy = py - DIRS[d].dy;
      if (px < 0 || py < 0 || px >= W || py >= L.H) continue;
      if (sx < 0 || sy < 0 || sx >= W || sy >= L.H) continue;
      const p = py * W + px, s = sy * W + sx;
      if (wall[p] || wall[s]) continue;
      if (dist[p] === -1) { dist[p] = dist[c] + 1; q.push(p); }
    }
  }
  return dist;
}

// ---------- push-space A* solver ----------
// opts: allowedBox (Uint8Array|null), allowedPlayer (Uint8Array|null), cap
function solvePush(L, dist, boxesInit, playerStart, opts) {
  const { W, H, N, wall, goal } = L;
  const allowedBox = opts.allowedBox || null;
  const allowedPlayer = opts.allowedPlayer || null;
  const cap = opts.cap;

  const boxSet = new Uint8Array(N);
  const seen = new Int32Array(N);
  const parent = new Int32Array(N);
  const parentDir = new Int8Array(N);
  let stamp = 0;

  // BFS over player-walkable cells (floor, allowed, no box). Fills parent for
  // path reconstruction. Returns {count, min} of the reachable region.
  function bfsPlayer(start, boxes) {
    stamp++;
    boxSet.fill(0);
    for (const b of boxes) boxSet[b] = 1;
    let count = 0, min = start;
    const stack = [start];
    seen[start] = stamp; parent[start] = -1;
    let head = 0;
    while (head < stack.length) {
      const c = stack[head++];
      count++;
      if (c < min) min = c;
      const cx = c % W, cy = (c / W) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        const ni = ny * W + nx;
        if (seen[ni] === stamp || wall[ni] || boxSet[ni]) continue;
        if (allowedPlayer && !allowedPlayer[ni]) continue;
        seen[ni] = stamp; parent[ni] = c; parentDir[ni] = d;
        stack.push(ni);
      }
    }
    return { count, min };
  }
  function walkPath(from, to) {
    // assumes bfsPlayer(from, ...) just ran and `to` was reached
    const chars = [];
    let c = to;
    while (c !== from) {
      const d = parentDir[c];
      chars.push(DIRS[d].ch);
      c = parent[c];
      if (c < 0) return null;
    }
    chars.reverse();
    return chars.join("");
  }
  function heuristic(boxes) {
    let h = 0;
    for (const b of boxes) {
      const d = dist[b];
      if (d < 0) return -1; // dead square
      h += d;
    }
    return h;
  }
  function isTunnel(cell, d) {
    const cx = cell % W, cy = (cell / W) | 0;
    // perpendicular neighbours
    const p1 = d < 2 ? [cx - 1, cy] : [cx, cy - 1];
    const p2 = d < 2 ? [cx + 1, cy] : [cx, cy + 1];
    const blocked = (p) => {
      if (p[0] < 0 || p[1] < 0 || p[0] >= W || p[1] >= H) return true;
      return wall[p[1] * W + p[0]] === 1;
    };
    return blocked(p1) && blocked(p2);
  }
  function dead22(boxes, moved) {
    // 2x2 block of walls/boxes containing a non-goal box is dead
    boxSet.fill(0);
    for (const b of boxes) boxSet[b] = 1;
    const mx = moved % W, my = (moved / W) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        const x0 = mx + ox, y0 = my + oy;
        if (x0 < 0 || y0 < 0 || x0 + 1 >= W || y0 + 1 >= H) continue;
        let allBlocked = true, badBox = false;
        for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
          const i = (y0 + dy) * W + (x0 + dx);
          if (wall[i]) continue;
          if (boxSet[i]) { if (!goal[i]) badBox = true; continue; }
          allBlocked = false;
        }
        if (allBlocked && badBox) return true;
      }
    }
    return false;
  }

  const h0 = heuristic(boxesInit);
  if (h0 < 0) return null;

  // node storage
  const nodes = [];
  const best = new Map();
  // binary heap of node indices keyed by (f, h, counter)
  const heap = [];
  let counter = 0;
  function less(a, b) {
    if (a.f !== b.f) return a.f < b.f;
    if (a.h !== b.h) return a.h < b.h;
    return a.c < b.c;
  }
  function hpush(item) {
    heap.push(item);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i]; heap[i] = heap[p]; heap[p] = t; i = p;
    }
  }
  function hpop() {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        let l = 2 * i + 1, r = l + 1, m = i;
        if (l < heap.length && less(heap[l], heap[m])) m = l;
        if (r < heap.length && less(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i]; heap[i] = heap[m]; heap[m] = t; i = m;
      }
    }
    return top;
  }

  const startRegion = bfsPlayer(playerStart, boxesInit);
  const startNode = {
    boxes: boxesInit.slice(), player: playerStart, canon: startRegion.min,
    g: 0, h: h0, parent: -1, seg: "", pushFrom: -1, pushDir: -1, pushSteps: 0,
  };
  nodes.push(startNode);
  best.set(startNode.canon + "|" + startNode.boxes.join(","), 0);
  hpush({ idx: 0, f: h0, h: h0, c: counter++ });

  let expansions = 0;
  while (heap.length) {
    const cur = hpop();
    const node = nodes[cur.idx];
    if (cur.f !== node.g + node.h) continue; // stale
    // goal test at pop time (optimality)
    let solved = true;
    for (const b of node.boxes) if (!goal[b]) { solved = false; break; }
    if (solved) {
      // reconstruct
      const segs = [];
      const pushes = [];
      let n = node;
      while (n.parent >= 0) {
        segs.push(n.seg);
        for (let s = 0; s < n.pushSteps; s++) {
          pushes.push({ from: n.pushFrom - 0, dir: n.pushDir, stepBack: s });
        }
        n = nodes[n.parent];
      }
      segs.reverse();
      // rebuild the push list in forward order: each node's macro pushes go
      // from pushFrom, then advance along dir per step
      const fwd = [];
      const chain = [];
      n = node;
      while (n.parent >= 0) { chain.push(n); n = nodes[n.parent]; }
      chain.reverse();
      for (const cn of chain) {
        let from = cn.pushFrom;
        for (let s = 0; s < cn.pushSteps; s++) {
          fwd.push({ from, dir: cn.pushDir });
          from += DIRS[cn.pushDir].dy * W + DIRS[cn.pushDir].dx;
        }
      }
      return { moves: segs.join(""), pushes: fwd, expansions };
    }
    if (++expansions > cap) return null;

    // player region + walk parents for this node
    bfsPlayer(node.player, node.boxes);
    const regionStamp = stamp;
    const regionParent = parent.slice();
    const regionParentDir = parentDir.slice();
    // snapshot reachability: successor BFS calls below overwrite `seen`
    const reach = new Uint8Array(N);
    for (let i = 0; i < N; i++) if (seen[i] === regionStamp) reach[i] = 1;

    const cands = [];
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const bx = b % W, by = (b / W) | 0;
      for (let d = 0; d < 4; d++) {
        const sx = bx - DIRS[d].dx, sy = by - DIRS[d].dy; // stance
        const tx = bx + DIRS[d].dx, ty = by + DIRS[d].dy; // dest
        if (sx < 0 || sy < 0 || sx >= W || sy >= H) continue;
        if (tx < 0 || ty < 0 || tx >= W || ty >= H) continue;
        const stance = sy * W + sx, dest = ty * W + tx;
        if (wall[dest] || (allowedBox && !allowedBox[dest])) continue;
        if (!reach[stance]) continue;
        // dest free of other boxes?
        let blocked = false;
        for (const ob of node.boxes) if (ob === dest) { blocked = true; break; }
        if (blocked) continue;
        if (dist[dest] < 0) continue; // dead square

        // ---- tunnel macro: extend straight pushes through 1-wide tunnels
        let steps = 1;
        let curBox = dest;
        let macroDead = false;
        const simBoxes = node.boxes.slice();
        simBoxes[bi] = dest;
        while (isTunnel(curBox, d) && !goal[curBox]) {
          const nx = (curBox % W) + DIRS[d].dx, ny = ((curBox / W) | 0) + DIRS[d].dy;
          if (nx < 0 || ny < 0 || nx >= W || ny >= H) { macroDead = true; break; }
          const nxt = ny * W + nx;
          if (wall[nxt] || (allowedBox && !allowedBox[nxt])) { macroDead = true; break; }
          let hasBox = false;
          for (const ob of simBoxes) if (ob === nxt) { hasBox = true; break; }
          if (hasBox) break; // stop macro in front of another box
          if (dist[nxt] < 0) { macroDead = true; break; }
          simBoxes[bi] = nxt;
          curBox = nxt;
          steps++;
        }
        if (macroDead) continue;

        const newBoxes = simBoxes.slice().sort((a, b2) => a - b2);
        if (dead22(newBoxes, curBox)) continue;
        const nh = heuristic(newBoxes);
        if (nh < 0) continue;
        const newPlayer = curBox - (DIRS[d].dy * W + DIRS[d].dx) * 1; // cell the box just left = final box pos - d... player ends at box's previous cell
        // player position after the (macro) push sequence: the cell occupied
        // by the box before its last step
        const lastFrom = curBox - (DIRS[d].dy * W + DIRS[d].dx);
        const reg = bfsPlayer(lastFrom, newBoxes);
        const key = reg.min + "|" + newBoxes.join(",");
        const ng = node.g + steps;
        if (best.has(key) && best.get(key) <= ng) continue;
        // walk path to stance using saved parents of the current node's BFS
        let wp = "";
        {
          const chars = [];
          let c = stance;
          let ok = true;
          while (c !== node.player) {
            const dd = regionParentDir[c];
            if (dd === undefined || dd < 0) { ok = false; break; }
            chars.push(DIRS[dd].ch);
            c = regionParent[c];
            if (c < 0) { ok = false; break; }
          }
          if (!ok) continue;
          chars.reverse();
          wp = chars.join("");
        }
        const seg = wp + DIRS[d].ch.repeat(steps);
        cands.push({
          boxes: newBoxes, player: lastFrom, canon: reg.min, g: ng, h: nh,
          seg, pushFrom: b, pushDir: d, pushSteps: steps,
          onGoal: goal[curBox] ? 0 : 1,
          dh: dist[curBox] - dist[b],
          regionSize: reg.count,
          bi, d,
        });
      }
    }
    // ---- push ordering: goal landings first, then biggest heuristic drop,
    // then largest resulting player region; fully deterministic tie-breaks.
    cands.sort((a, b2) =>
      (a.onGoal - b2.onGoal) ||
      (a.dh - b2.dh) ||
      (b2.regionSize - a.regionSize) ||
      (a.bi - b2.bi) ||
      (a.d - b2.d));
    for (const cd of cands) {
      const key = cd.canon + "|" + cd.boxes.join(",");
      if (best.has(key) && best.get(key) <= cd.g) continue;
      best.set(key, cd.g);
      const idx = nodes.length;
      nodes.push({
        boxes: cd.boxes, player: cd.player, canon: cd.canon, g: cd.g, h: cd.h,
        parent: cur.idx, seg: cd.seg, pushFrom: cd.pushFrom, pushDir: cd.pushDir,
        pushSteps: cd.pushSteps,
      });
      hpush({ idx, f: cd.g + cd.h, h: cd.h, c: counter++ });
    }
  }
  return null;
}

// ---------- replay / simulation helpers (also used as internal check) ----------
function simulate(L, moves) {
  const { W, H, wall, goal } = L;
  const boxes = new Set(L.boxes);
  let p = L.player;
  let pushes = 0;
  for (const ch of moves) {
    const d = DIRS.findIndex((x) => x.ch === ch);
    if (d < 0) continue;
    const nx = (p % W) + DIRS[d].dx, ny = ((p / W) | 0) + DIRS[d].dy;
    if (nx < 0 || ny < 0 || nx >= W || ny >= H) return null;
    const ni = ny * W + nx;
    if (wall[ni]) return null;
    if (boxes.has(ni)) {
      const bx = nx + DIRS[d].dx, by = ny + DIRS[d].dy;
      if (bx < 0 || by < 0 || bx >= W || by >= H) return null;
      const bi2 = by * W + bx;
      if (wall[bi2] || boxes.has(bi2)) return null;
      boxes.delete(ni); boxes.add(bi2); pushes++;
    }
    p = ni;
  }
  let ok = true;
  for (const b of boxes) if (!goal[b]) ok = false;
  return { ok, pushes, boxes: [...boxes].sort((a, b) => a - b), player: p };
}

// BFS walk on the full level (boxes as obstacles); returns move string or null
function walkPathGlobal(L, from, to, boxSetArr) {
  const { W, H, wall } = L;
  const boxSet = new Set(boxSetArr);
  const prev = new Map();
  const pd = new Map();
  const q = [from];
  prev.set(from, -1);
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    if (c === to) break;
    const cx = c % W, cy = (c / W) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const ni = ny * W + nx;
      if (prev.has(ni) || wall[ni] || boxSet.has(ni)) continue;
      prev.set(ni, c); pd.set(ni, d); q.push(ni);
    }
  }
  if (!prev.has(to)) return null;
  const chars = [];
  let c = to;
  while (c !== from) { chars.push(DIRS[pd.get(c)].ch); c = prev.get(c); }
  chars.reverse();
  return chars.join("");
}

// ---------- whole-level solve with room decomposition ----------
function solveLevel(L) {
  const dist = computeDist(L);
  const { W, H, N, wall } = L;
  const playerRow = (L.player / W) | 0;

  // hallway = the player's row
  const hallway = new Uint8Array(N);
  let hallwayOk = L.player >= 0;
  for (let x = 0; x < W; x++) {
    const i = playerRow * W + x;
    if (!wall[i]) hallway[i] = 1;
  }
  // components of floor minus the hallway row
  const comp = new Int32Array(N).fill(-1);
  let ncomp = 0;
  for (let i = 0; i < N; i++) {
    if (wall[i] || hallway[i] || comp[i] >= 0) continue;
    const stack = [i];
    comp[i] = ncomp;
    while (stack.length) {
      const c = stack.pop();
      const cx = c % W, cy = (c / W) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        const ni = ny * W + nx;
        if (wall[ni] || hallway[ni] || comp[ni] >= 0) continue;
        comp[ni] = ncomp; stack.push(ni);
      }
    }
    ncomp++;
  }
  const compBoxes = new Map();
  const compGoals = new Map();
  let decompOk = hallwayOk && ncomp > 0;
  for (const b of L.boxes) {
    if (comp[b] < 0) { decompOk = false; break; }
    if (!compBoxes.has(comp[b])) compBoxes.set(comp[b], []);
    compBoxes.get(comp[b]).push(b);
  }
  if (decompOk) {
    for (let i = 0; i < N; i++) if (L.goal[i] && comp[i] >= 0) {
      if (!compGoals.has(comp[i])) compGoals.set(comp[i], 0);
      compGoals.set(comp[i], compGoals.get(comp[i]) + 1);
    }
    for (const [cid, bs] of compBoxes) {
      if ((compGoals.get(cid) || 0) !== bs.length) { decompOk = false; break; }
    }
  }

  if (decompOk) {
    const rooms = [...compBoxes.entries()].map(([cid, bs]) => ({
      cid, boxes: bs.slice().sort((a, b) => a - b), minBox: Math.min(...bs),
    })).sort((a, b) => a.minBox - b.minBox);

    const roomPushes = [];
    let allOk = true;
    for (const room of rooms) {
      const allowedBox = new Uint8Array(N);
      const allowedPlayer = new Uint8Array(N);
      for (let i = 0; i < N; i++) {
        if (comp[i] === room.cid) { allowedBox[i] = 1; allowedPlayer[i] = 1; }
        if (hallway[i]) allowedPlayer[i] = 1;
      }
      const sol = solvePush(L, dist, room.boxes, L.player, {
        allowedBox, allowedPlayer, cap: 1500000,
      });
      if (!sol) { allOk = false; break; }
      roomPushes.push(sol.pushes);
    }
    if (allOk) {
      // concatenate: walk through the hallway between rooms, replay pushes
      let boxes = L.boxes.slice();
      let player = L.player;
      let moves = "";
      let execOk = true;
      for (const pushes of roomPushes) {
        for (const pu of pushes) {
          const stance = pu.from - (DIRS[pu.dir].dy * W + DIRS[pu.dir].dx);
          const wp = walkPathGlobal(L, player, stance, boxes);
          if (wp === null) { execOk = false; break; }
          moves += wp + DIRS[pu.dir].ch;
          const dest = pu.from + (DIRS[pu.dir].dy * W + DIRS[pu.dir].dx);
          boxes = boxes.map((b) => (b === pu.from ? dest : b)).sort((a, b) => a - b);
          player = pu.from;
        }
        if (!execOk) break;
      }
      if (execOk) {
        const chk = simulate(L, moves);
        if (chk && chk.ok) return moves;
      }
    }
  }
  // fallback: whole-level push A*
  const sol = solvePush(L, dist, L.boxes, L.player, { cap: 2500000 });
  if (sol) {
    const chk = simulate(L, sol.moves);
    if (chk && chk.ok) return sol.moves;
  }
  return "";
}

// ---------- main ----------
function main() {
  const inDir = process.env.MAKEFUN_INPUT_DIR ?? "/input";
  const outDir = process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
  let ids = [];
  try {
    ids = readdirSync(inDir).filter((e) => {
      try { return statSync(join(inDir, e)).isDirectory(); } catch { return false; }
    }).sort();
  } catch { ids = []; }
  for (const id of ids) {
    const lvlPath = join(inDir, id, "level.txt");
    let moves = "";
    if (existsSync(lvlPath)) {
      try {
        const L = parseLevel(readFileSync(lvlPath, "utf8"));
        if (L.player >= 0 && L.boxes.length) moves = solveLevel(L);
      } catch { moves = ""; }
    }
    const od = join(outDir, id);
    mkdirSync(od, { recursive: true });
    writeFileSync(join(od, "solution.txt"), moves ? moves + "\n" : "");
  }
}
main();
