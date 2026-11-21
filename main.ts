// Sokoban solver: push-optimal per room, then stitched.
// SPDX-License-Identifier: MIT
//
// The generated levels are a hallway with one-square-wide doors into rooms of
// 3-4 boxes. A box pushed through a door can never come back, so every room is
// independent: the push-optimal total is the sum of the rooms' optima. Each
// room is solved with A* over pushes (state = sorted box cells + the player's
// reachable region), a min-assignment heuristic over BFS distances, dead-square
// pruning (reverse pull reachability) and 2x2 freeze pruning. Walks between
// pushes and between rooms are shortest BFS paths (walks cost no pushes, so
// only the push count matters). Everything is budgeted by expansions, never by
// the clock, so two runs write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets (deterministic: counted, never timed).
const ROOM_ASTAR_CAP = 150_000;
const ROOM_GREEDY_CAP = 80_000;
const GLOBAL_ASTAR_CAP = 200_000;

const DIRS: { dx: number; dy: number; ch: string }[] = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

interface Level {
  w: number;
  h: number;
  wall: boolean[];
  goals: Set<number>;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const raw = text.split('\n');
  if (raw.length > 0 && raw[raw.length - 1] === '') raw.pop();
  const h = raw.length;
  const w = Math.max(...raw.map((r) => r.length));
  const wall: boolean[] = new Array(w * h).fill(true);
  const goals = new Set<number>();
  const boxes: number[] = [];
  let player = 0;
  raw.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = false;
      if (ch === '.' || ch === '*' || ch === '+') goals.add(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/** BFS distances from `from` over cells where `open(i)` is true. */
function bfsDist(from: number, w: number, open: (i: number) => boolean): Map<number, number> {
  const dist = new Map<number, number>([[from, 0]]);
  const q = [from];
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    const d = dist.get(c)! + 1;
    const x = c % w;
    const y = (c / w) | 0;
    const nb = [
      y > 0 ? c - w : -1,
      c + w,
      x > 0 ? c - 1 : -1,
      c + 1,
    ];
    for (const n of nb) {
      if (n < 0 || dist.has(n) || !open(n)) continue;
      dist.set(n, d);
      q.push(n);
    }
  }
  return dist;
}

/** Shortest walk path (letters) from `from` to `to`, or null. Deterministic. */
function walkPath(from: number, to: number, w: number, h: number, blocked: (i: number) => boolean): string | null {
  if (from === to) return '';
  const prev = new Map<number, { p: number; ch: string }>([[from, { p: -1, ch: '' }]]);
  const q = [from];
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    for (const dir of DIRS) {
      const x = c % w;
      const y = (c / w) | 0;
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (prev.has(n) || blocked(n)) continue;
      prev.set(n, { p: c, ch: dir.ch });
      if (n === to) {
        let s = '';
        let cur = n;
        while (cur !== from) {
          const e = prev.get(cur)!;
          s = e.ch + s;
          cur = e.p;
        }
        return s;
      }
      q.push(n);
    }
  }
  return null;
}

/** Min-cost assignment of boxes to goals using precomputed distances (Infinity when impossible). */
function assignmentBound(boxes: number[], goals: number[], dist: number[][]): number {
  const n = boxes.length;
  const m = goals.length;
  if (n === 0) return 0;
  if (n > m) return Infinity;
  // Brute-force permutations (n <= 4 in practice; guard anyway).
  let best = Infinity;
  const used = new Array(m).fill(false);
  const rec = (i: number, acc: number): void => {
    if (acc >= best) return;
    if (i === n) {
      best = acc;
      return;
    }
    for (let j = 0; j < m; j++) {
      if (used[j]) continue;
      const d = dist[i]![j]!;
      if (!Number.isFinite(d)) continue;
      used[j] = true;
      rec(i + 1, acc + d);
      used[j] = false;
    }
  };
  rec(0, 0);
  return best;
}

interface Push {
  box: number;
  dir: number; // index into DIRS
}

interface RoomSolution {
  pushes: Push[];
  endPlayer: number;
}

/**
 * Push-optimal solve of one room.
 * `cells`: room floor cells (boxes may only ever be here). `door`: the hallway
 * cell the player enters through. `goals`/`boxes0`: room goals/boxes.
 * `player0`: starting player cell (the door). `needExit`: when true the goal
 * test also requires the player to be able to reach the door afterwards.
 */
function solveRoom(
  cells: number[],
  door: number,
  goals: number[],
  boxes0: number[],
  player0: number,
  w: number,
  h: number,
  needExit: boolean,
  cap: number,
  greedy: boolean,
): RoomSolution | null {
  const cellSet = new Set(cells);
  const inRoom = (i: number): boolean => cellSet.has(i);
  const playerOpen = (i: number): boolean => cellSet.has(i) || i === door;
  const goalSet = new Set(goals);
  const n = boxes0.length;

  // BFS distances between room cells (ignoring boxes), for the heuristic.
  const distFrom = new Map<number, Map<number, number>>();
  for (const c of cells) distFrom.set(c, bfsDist(c, w, inRoom));
  // dist[boxIdx][goalIdx] filled per state from these maps.
  const boxGoalDist = (b: number, g: number): number => distFrom.get(b)?.get(g) ?? Infinity;

  // Dead squares: reverse pull reachability. A cell is live when a box there
  // could be pulled (ignoring whether the player can actually get into place)
  // step by step onto a goal. Boxes on non-live cells are pruned.
  const live = new Set<number>(goals);
  let changed = true;
  while (changed) {
    changed = false;
    for (const c of cells) {
      if (live.has(c)) continue;
      const x = c % w;
      const y = (c / w) | 0;
      for (const dir of DIRS) {
        const nx = x + dir.dx;
        const ny = y + dir.dy;
        const bx = x - dir.dx;
        const by = y - dir.dy;
        const to = ny * w + nx;
        const stand = by * w + bx;
        if (live.has(to) && playerOpen(stand)) {
          live.add(c);
          changed = true;
          break;
        }
      }
    }
  }

  const isDead = (boxes: number[]): boolean => {
    const boxSet = new Set(boxes);
    for (const b of boxes) {
      if (!goalSet.has(b) && !live.has(b)) return true;
    }
    // 2x2 freeze: a 2x2 block of walls/boxes holding a box off a goal.
    for (const b of boxes) {
      if (goalSet.has(b)) continue;
      const x = b % w;
      const y = (b / w) | 0;
      const blocks: [number, number][] = [
        [x - 1, y - 1],
        [x, y - 1],
        [x - 1, y],
        [x, y],
      ];
      for (const [ox, oy] of blocks) {
        if (ox < 0 || oy < 0 || ox + 1 >= w) continue;
        const cs = [oy * w + ox, oy * w + ox + 1, (oy + 1) * w + ox, (oy + 1) * w + ox + 1];
        let allBlocked = true;
        let hasBox = false;
        for (const c of cs) {
          if (boxSet.has(c)) {
            hasBox = true;
          } else if (playerOpen(c)) {
            allBlocked = false;
            break;
          }
        }
        if (allBlocked && hasBox) return true;
      }
    }
    return false;
  };

  // Flood fill of the player's reachable region from `from`, boxes as walls.
  const region = (from: number, boxes: number[]): Set<number> => {
    const boxSet = new Set(boxes);
    const seen = new Set<number>([from]);
    const q = [from];
    for (let head = 0; head < q.length; head++) {
      const c = q[head]!;
      const x = c % w;
      const y = (c / w) | 0;
      for (const dir of DIRS) {
        const nx = x + dir.dx;
        const ny = y + dir.dy;
        if (nx < 0 || ny < 0 || nx >= w) continue;
        const nb = ny * w + nx;
        if (seen.has(nb) || boxSet.has(nb) || !playerOpen(nb)) continue;
        seen.add(nb);
        q.push(nb);
      }
    }
    return seen;
  };

  const regionRep = (r: Set<number>): number => {
    let m = Infinity;
    for (const c of r) if (c < m) m = c;
    return m;
  };

  // A* / greedy state store.
  const sBoxes: number[][] = [boxes0.slice()];
  const sPlayer: number[] = [player0];
  const sG: number[] = [0];
  const sParent: number[] = [-1];
  const sPush: (Push | null)[] = [null];
  const best = new Map<string, number>();
  const keyOf = (boxes: number[], rep: number): string => boxes.join(',') + '|' + rep;
  best.set(keyOf(boxes0, regionRep(region(player0, boxes0))), 0);

  // Binary heap on [f, h, seq]; deterministic tie-breaking.
  const heap: [number, number, number, number][] = [];
  const heapLess = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const hOf = (boxes: number[]): number => {
    const rows: number[][] = boxes.map((b) => goals.map((g) => boxGoalDist(b, g)));
    return assignmentBound(boxes, goals, rows);
  };

  const h0 = hOf(boxes0);
  if (!Number.isFinite(h0)) return null;
  let seq = 0;
  heapPush([greedy ? h0 : h0, h0, seq++, 0]);

  let expanded = 0;
  while (heap.length > 0 && expanded < cap) {
    const [, , , s] = heapPop();
    const boxes = sBoxes[s]!;
    const g = sG[s]!;
    const rep = regionRep(region(sPlayer[s]!, boxes));
    if (best.get(keyOf(boxes, rep)) !== s) continue;
    if (boxes.every((b) => goalSet.has(b))) {
      if (!needExit || region(sPlayer[s]!, boxes).has(door)) {
        const pushes: Push[] = [];
        for (let t = s; t > 0; t = sParent[t]!) pushes.push(sPush[t]!);
        pushes.reverse();
        return { pushes, endPlayer: sPlayer[s]! };
      }
    }
    expanded++;
    const reg = region(sPlayer[s]!, boxes);
    const boxSet = new Set(boxes);
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (let di = 0; di < 4; di++) {
        const dir = DIRS[di]!;
        const nx = bx + dir.dx;
        const ny = by + dir.dy;
        const tx = bx - dir.dx;
        const ty = by - dir.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const target = ny * w + nx;
        const stand = ty * w + tx;
        if (!inRoom(target) || boxSet.has(target)) continue;
        if (!reg.has(stand)) continue;
        const nb = boxes.slice();
        nb[bi] = target;
        nb.sort((a, b2) => a - b2);
        if (isDead(nb)) continue;
        const hv = hOf(nb);
        if (!Number.isFinite(hv)) continue;
        const ng = g + 1;
        const nrep = regionRep(region(b, nb));
        const k = keyOf(nb, nrep);
        const prev = best.get(k);
        if (prev !== undefined && sG[prev]! <= ng) continue;
        const t = sBoxes.length;
        sBoxes.push(nb);
        sPlayer.push(b);
        sG.push(ng);
        sParent.push(s);
        sPush.push({ box: b, dir: di });
        best.set(k, t);
        heapPush([greedy ? hv : ng + hv, hv, seq++, t]);
      }
    }
  }
  return null;
}

/** Global move-based A* fallback (fewest moves), used when decomposition fails. */
function solveGlobal(lv: Level): string {
  const { w, h, wall, goals, boxes, player } = lv;
  const dist = (b: number): number => {
    let m = Infinity;
    for (const g of goals) {
      const d = Math.abs((g % w) - (b % w)) + Math.abs(((g / w) | 0) - ((b / w) | 0));
      if (d < m) m = d;
    }
    return m;
  };
  const est = (bs: number[]): number => bs.reduce((a, b) => a + dist(b), 0);
  const key = (p: number, bs: number[]): string => p + '|' + bs.join(',');
  const sp: number[] = [player];
  const sb: number[][] = [boxes.slice()];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const step: string[] = [''];
  const best = new Map<string, number>([[key(player, boxes), 0]]);
  const heap: [number, number, number][] = [[est(boxes), 0, 0]];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const push = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const pop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  let expanded = 0;
  while (heap.length > 0 && expanded < GLOBAL_ASTAR_CAP) {
    const [, g, s] = pop();
    if (best.get(key(sp[s]!, sb[s]!)) !== s) continue;
    const bs = sb[s]!;
    if (bs.every((b) => goals.has(b))) {
      let moves = '';
      for (let t = s; t > 0; t = parent[t]!) moves = step[t] + moves;
      return moves;
    }
    expanded++;
    const p = sp[s]!;
    const px = p % w;
    const py = (p / w) | 0;
    for (const dir of DIRS) {
      const nx = px + dir.dx;
      const ny = py + dir.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const q = ny * w + nx;
      if (wall[q]) continue;
      let nb = bs;
      let move = dir.ch;
      const at = bs.indexOf(q);
      if (at >= 0) {
        const tx = nx + dir.dx;
        const ty = ny + dir.dy;
        if (tx < 0 || ty < 0 || tx >= w || ty >= h) continue;
        const t2 = ty * w + tx;
        if (wall[t2] || bs.includes(t2)) continue;
        nb = bs.slice();
        nb[at] = t2;
        nb.sort((a, b2) => a - b2);
        move = dir.ch.toUpperCase();
      }
      const k = key(q, nb);
      const seen = best.get(k);
      if (seen !== undefined && sg[seen]! <= g + 1) continue;
      const t = sp.length;
      sp.push(q);
      sb.push(nb);
      sg.push(g + 1);
      parent.push(s);
      step.push(move);
      best.set(k, t);
      push([g + 1 + est(nb), g + 1, t]);
    }
  }
  return '';
}

function solveLevel(text: string): string {
  const lv = parseLevel(text);
  const { w, h, wall, goals, boxes, player } = lv;
  const hallRow = ((player / w) | 0);

  // Rooms: connected components of floor cells excluding the hallway row.
  const seen = new Set<number>();
  const rooms: { cells: number[]; door: number }[] = [];
  let bad = false;
  for (let y = 0; y < h && !bad; y++) {
    for (let x = 0; x < w; x++) {
      const i = y * w + x;
      if (y === hallRow || wall[i] || seen.has(i)) continue;
      const comp: number[] = [];
      const stack = [i];
      seen.add(i);
      while (stack.length > 0) {
        const c = stack.pop()!;
        comp.push(c);
        const cx = c % w;
        const cy = (c / w) | 0;
        for (const dir of DIRS) {
          const nx = cx + dir.dx;
          const ny = cy + dir.dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h || ny === hallRow) continue;
          const n = ny * w + nx;
          if (wall[n] || seen.has(n)) continue;
          seen.add(n);
          stack.push(n);
        }
      }
      // Door: hallway cells adjacent to this component.
      const doorCount = new Map<number, number>();
      for (const c of comp) {
        const cx = c % w;
        const cy = (c / w) | 0;
        for (const dir of DIRS) {
          const nx = cx + dir.dx;
          const ny = cy + dir.dy;
          if (ny === hallRow && nx >= 0 && nx < w && !wall[ny * w + nx]) {
            const d = ny * w + nx;
            doorCount.set(d, (doorCount.get(d) ?? 0) + 1);
          }
        }
      }
      const roomBoxes = comp.filter((c) => boxes.includes(c));
      const roomGoals = comp.filter((c) => goals.has(c));
      if (doorCount.size === 0 || roomBoxes.length === 0 || roomBoxes.length !== roomGoals.length || roomBoxes.length > 4) {
        bad = true;
        break;
      }
      let door = -1;
      let doorN = -1;
      for (const [d, n2] of [...doorCount.entries()].sort((a, b) => a[0] - b[0])) {
        if (n2 > doorN) {
          doorN = n2;
          door = d;
        }
      }
      rooms.push({ cells: comp.sort((a, b) => a - b), door });
    }
  }
  // Any box in the hallway row, or a goal/box outside rooms, breaks the model.
  const roomCells = new Set(rooms.flatMap((r) => r.cells));
  if (!bad) {
    for (const b of boxes) {
      if (((b / w) | 0) === hallRow || !roomCells.has(b)) {
        bad = true;
        break;
      }
    }
    for (const g of goals) {
      if (((g / w) | 0) === hallRow || !roomCells.has(g)) {
        bad = true;
        break;
      }
    }
  }
  if (bad || rooms.length === 0) return solveGlobal(lv);

  // Solve rooms nearest-first from the player's current cell.
  const remaining = rooms.slice();
  let cur = player;
  let curBoxes = boxes.slice();
  let moves = '';
  const blocked = (): ((i: number) => boolean) => {
    const boxSet = new Set(curBoxes);
    return (i: number): boolean => wall[i] || boxSet.has(i);
  };
  while (remaining.length > 0) {
    // Pick the room whose door is nearest by walk distance.
    let bestIdx = -1;
    let bestWalk: string | null = null;
    let bestLen = Infinity;
    for (let ri = 0; ri < remaining.length; ri++) {
      const r = remaining[ri]!;
      const walk = walkPath(cur, r.door, w, h, blocked());
      if (walk === null) continue;
      if (walk.length < bestLen || (walk.length === bestLen && (bestIdx < 0 || r.door < remaining[bestIdx]!.door))) {
        bestLen = walk.length;
        bestWalk = walk;
        bestIdx = ri;
      }
    }
    if (bestIdx < 0) return solveGlobal(lv);
    const room = remaining.splice(bestIdx, 1)[0]!;
    const walkToDoor = bestWalk ?? '';
    const roomGoals = room.cells.filter((c) => goals.has(c));
    const roomBoxes = room.cells.filter((c) => curBoxes.includes(c)).sort((a, b) => a - b);
    const tryRoom = (needExit: boolean, cap: number, greedy: boolean) => {
      const s = solveRoom(room.cells, room.door, roomGoals, roomBoxes, room.door, w, h, needExit, cap, greedy);
      if (!s) return null;
      return pushesToMoves(s, room, curBoxes, lv);
    };
    // First try push-optimal without an exit requirement; only if the player
    // ends up unable to leave the room, re-solve requiring the exit (which
    // may cost extra pushes). Nothing is committed until a usable result exists.
    let res = tryRoom(false, ROOM_ASTAR_CAP, false);
    if (!res || !res.canExit) {
      res = tryRoom(true, ROOM_ASTAR_CAP, false);
      if (!res) res = tryRoom(true, ROOM_GREEDY_CAP, true);
      if (!res) return solveGlobal(lv);
    }
    moves += walkToDoor + res.moves;
    cur = res.endPlayer;
    curBoxes = res.boxes;
  }
  return moves;
}

/** Turn a room's push list into walk+push letters; returns null when a walk fails. */
function pushesToMoves(
  sol: RoomSolution,
  room: { cells: number[]; door: number },
  curBoxes: number[],
  lv: Level,
): { moves: string; endPlayer: number; boxes: number[]; canExit: boolean } | null {
  const { w, h, wall } = lv;
  const cellSet = new Set(room.cells);
  const boxes = curBoxes.slice();
  let player = room.door;
  let moves = '';
  for (const p of sol.pushes) {
    const dir = DIRS[p.dir]!;
    const bx = p.box % w;
    const by = (p.box / w) | 0;
    const stand = (by - dir.dy) * w + (bx - dir.dx);
    const boxSet = new Set(boxes);
    const walk = walkPath(player, stand, w, h, (i) => wall[i] || boxSet.has(i) || (!cellSet.has(i) && i !== room.door));
    if (walk === null) return null;
    moves += walk + dir.ch;
    const at = boxes.indexOf(p.box);
    boxes[at] = (by + dir.dy) * w + (bx + dir.dx);
    boxes.sort((a, b) => a - b);
    player = p.box;
  }
  // Can the player reach the door from where the last push left them?
  const boxSet = new Set(boxes);
  const canExit = walkPath(player, room.door, w, h, (i) => wall[i] || boxSet.has(i) || (!cellSet.has(i) && i !== room.door)) !== null;
  return { moves, endPlayer: player, boxes, canExit };
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
