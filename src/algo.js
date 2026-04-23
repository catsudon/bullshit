/**
 * algo.js — Core algorithm implementations
 * 
 * 1. Standard Chaikin's Algorithm
 * 2. Collision-Aware Chaikin (Constraint-Based)
 * 3. Geometry helpers (SAT, circle vs segment, AABB vs segment)
 */

'use strict';

/* ─── Geometry Utilities ────────────────────────────────────────────────────── */

/**
 * Clamp value between min and max
 */
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/**
 * Dot product of two 2D vectors
 */
const dot = (a, b) => a.x * b.x + a.y * b.y;

/**
 * Vector subtraction
 */
const vsub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y });

/**
 * Vector addition
 */
const vadd = (a, b) => ({ x: a.x + b.x, y: a.y + b.y });

/**
 * Scalar multiply
 */
const vscale = (v, s) => ({ x: v.x * s, y: v.y * s });

/**
 * Vector magnitude
 */
const vmag = (v) => Math.sqrt(v.x * v.x + v.y * v.y);

/**
 * Normalize a vector (returns zero-length safe result)
 */
const vnorm = (v) => {
  const m = vmag(v);
  return m < 1e-9 ? { x: 0, y: 0 } : { x: v.x / m, y: v.y / m };
};

/**
 * Linear interpolate between two points
 */
const lerp2 = (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });

/**
 * Deep-copy an array of {x,y} points
 */
const copyPath = (pts) => pts.map(p => ({ x: p.x, y: p.y }));

/* ─── Obstacle Definitions ───────────────────────────────────────────────────── */

/**
 * CircleObstacle: { type:'circle', x, y, r }
 * RectObstacle:   { type:'rect', x, y, w, h }
 */

/**
 * Test if point P is inside a circle obstacle.
 * Returns { inside, depth, normal } where normal points FROM center TO point.
 */
function testPointCircle(px, py, obs) {
  const dx = px - obs.x;
  const dy = py - obs.y;
  const dist = Math.sqrt(dx * dx + dy * dy);
  const inside = dist < obs.r;
  const depth = inside ? (obs.r - dist) : 0;
  const normal = dist < 1e-6
    ? { x: 1, y: 0 }
    : { x: dx / dist, y: dy / dist };
  return { inside, depth, normal };
}

/**
 * Test if point P is inside a rectangle obstacle.
 * Returns { inside, depth, normal } where normal points out of the rect surface.
 */
function testPointRect(px, py, obs) {
  const lx = px - obs.x;
  const ly = py - obs.y;
  if (lx < 0 || lx > obs.w || ly < 0 || ly > obs.h) {
    return { inside: false, depth: 0, normal: { x: 0, y: 0 } };
  }
  // Find minimum penetration axis
  const dl = lx;
  const dr = obs.w - lx;
  const dt = ly;
  const db = obs.h - ly;
  const minD = Math.min(dl, dr, dt, db);
  let nx = 0, ny = 0;
  if (minD === dl) { nx = -1; }
  else if (minD === dr) { nx = 1; }
  else if (minD === dt) { ny = -1; }
  else { ny = 1; }
  return { inside: true, depth: minD, normal: { x: nx, y: ny } };
}

/**
 * Test point against any obstacle (dispatches by type)
 */
function testPoint(px, py, obs) {
  if (obs.type === 'circle') return testPointCircle(px, py, obs);
  if (obs.type === 'rect')   return testPointRect(px, py, obs);
  return { inside: false, depth: 0, normal: { x: 0, y: 0 } };
}

/**
 * Count how many obstacle violations a path has.
 * Returns an array of { index, obstacle, result } for each violation.
 */
function findCollisions(path, obstacles) {
  const violations = [];
  for (let i = 0; i < path.length; i++) {
    const p = path[i];
    for (const obs of obstacles) {
      const res = testPoint(p.x, p.y, obs);
      if (res.inside) {
        violations.push({ index: i, obstacle: obs, result: res, x: p.x, y: p.y });
      }
    }
  }
  return violations;
}

/**
 * Test if segment (A→B) intersects a circle obstacle.
 * Returns { hit, tEntry, tExit } where tEntry/tExit are parametric [0,1].
 */
function segmentCircle(ax, ay, bx, by, obs) {
  const dx = bx - ax, dy = by - ay;
  const fx = ax - obs.x, fy = ay - obs.y;
  const a = dx * dx + dy * dy;
  const b = 2 * (fx * dx + fy * dy);
  const c = fx * fx + fy * fy - obs.r * obs.r;
  let discriminant = b * b - 4 * a * c;
  if (discriminant < 0 || a < 1e-12) return { hit: false };
  discriminant = Math.sqrt(discriminant);
  const t1 = (-b - discriminant) / (2 * a);
  const t2 = (-b + discriminant) / (2 * a);
  if (t1 <= 1 && t2 >= 0) {
    return { hit: true, tEntry: Math.max(t1, 0), tExit: Math.min(t2, 1) };
  }
  return { hit: false };
}

/**
 * Test if segment (A→B) intersects a rectangle obstacle.
 * Uses Liang-Barsky parametric clipping.
 * Returns { hit, tEntry, tExit }.
 */
function segmentVsRect(ax, ay, bx, by, obs) {
  const dx = bx - ax, dy = by - ay;
  // p[i] * t <= q[i] for each of 4 half-planes
  const p = [-dx,  dx, -dy,  dy];
  const q = [
    ax - obs.x,
    obs.x + obs.w - ax,
    ay - obs.y,
    obs.y + obs.h - ay,
  ];
  let tEntry = 0, tExit = 1;
  for (let i = 0; i < 4; i++) {
    if (Math.abs(p[i]) < 1e-10) {
      // Parallel to slab — reject if outside
      if (q[i] < 0) return { hit: false };
    } else {
      const t = q[i] / p[i];
      if (p[i] < 0) { if (t > tEntry) tEntry = t; }
      else          { if (t < tExit)  tExit  = t; }
    }
    if (tEntry > tExit) return { hit: false };
  }
  return { hit: tEntry < tExit, tEntry, tExit };
}

/**
 * Unified segment-vs-obstacle test.
 */
function segmentVsObstacle(ax, ay, bx, by, obs) {
  if (obs.type === 'circle') return segmentCircle(ax, ay, bx, by, obs);
  if (obs.type === 'rect')   return segmentVsRect(ax, ay, bx, by, obs);
  return { hit: false };
}

/**
 * Edge resolution — iterative version.
 *
 * A single midpoint insertion is not enough for wide obstacles:
 *   A ──────────[rect]────────── B
 * Inserting pushed midpoint M gives:  A → M → B
 * but A→M and M→B can STILL cross the rect.
 *
 * Fix: repeat the scan+insert cycle until the full path has zero
 * edge-obstacle crossings (converged), capped at MAX_PASSES to prevent
 * infinite loops in degenerate geometry.
 *
 * @param {Array<{x,y}>} path
 * @param {Array}        obstacles
 * @param {number}       pushStrength
 * @param {number}       maxIter      - vertex push iterations
 * @returns {Array<{x,y}>}  resolved path (may have extra vertices)
 */
function resolveEdges(path, obstacles, pushStrength, maxIter) {
  const MAX_PASSES = 32; // safety cap — each pass at least halves uncrossed length

  for (let pass = 0; pass < MAX_PASSES; pass++) {
    const insertions = [];

    for (let i = 0; i < path.length - 1; i++) {
      const A = path[i], B = path[i + 1];
      for (const obs of obstacles) {
        const hit = segmentVsObstacle(A.x, A.y, B.x, B.y, obs);
        if (!hit.hit) continue;
        // Midpoint of the intersection chord (guaranteed inside obstacle)
        const tMid = (hit.tEntry + hit.tExit) / 2;
        const mx = A.x + (B.x - A.x) * tMid;
        const my = A.y + (B.y - A.y) * tMid;
        // Push it out of all obstacles
        const { pt } = resolveVertex({ x: mx, y: my }, obstacles, pushStrength, maxIter);
        insertions.push({ after: i, pt });
        break; // one insertion per edge, re-scan next pass
      }
    }

    // No crossings found — path is fully clear
    if (insertions.length === 0) break;

    // Apply insertions in reverse so earlier indices stay valid
    for (let k = insertions.length - 1; k >= 0; k--) {
      path.splice(insertions[k].after + 1, 0, insertions[k].pt);
    }
  }
  return path;
}

/* ─── Standard Chaikin's Algorithm ─────────────────────────────────────────── */

/**
 * One pass of Chaikin corner-cutting.
 * @param {Array<{x,y}>} pts - input control points
 * @param {number} alpha     - cut ratio [0.05, 0.49], default 0.25
 * @returns {Array<{x,y}>}   - new point list (approx 2x longer)
 */
function chaikinPass(pts, alpha = 0.25) {
  if (pts.length < 2) return copyPath(pts);
  const out = [];
  // Keep first point
  out.push({ x: pts[0].x, y: pts[0].y });
  for (let i = 0; i < pts.length - 1; i++) {
    const p0 = pts[i], p1 = pts[i + 1];
    const q = lerp2(p0, p1, alpha);       // Q = 1/4 along edge
    const r = lerp2(p0, p1, 1 - alpha);  // R = 3/4 along edge
    out.push(q, r);
  }
  // Keep last point
  out.push({ x: pts[pts.length - 1].x, y: pts[pts.length - 1].y });
  return out;
}

/**
 * Full Standard Chaikin smoothing (n iterations)
 * @param {Array<{x,y}>} controlPoints
 * @param {number} iterations
 * @param {number} alpha
 * @returns {{ path: Array<{x,y}>, timeUs: number }}
 */
function standardChaikin(controlPoints, iterations = 4, alpha = 0.25) {
  const t0 = performance.now();
  let path = copyPath(controlPoints);
  for (let i = 0; i < iterations; i++) {
    path = chaikinPass(path, alpha);
  }
  const timeUs = (performance.now() - t0) * 1000;
  return { path, timeUs };
}

/* ─── Collision-Aware Chaikin ────────────────────────────────────────────────── */

/**
 * Push a single vertex out of all penetrating obstacles.
 * @param {{x,y}} pt         - vertex to resolve
 * @param {Array} obstacles
 * @param {number} pushStrength
 * @param {number} maxIter   - max constraint-solving iterations
 * @returns {{ pt: {x,y}, pushCount: number, resolved: boolean }}
 */
function resolveVertex(pt, obstacles, pushStrength = 1.0, maxIter = 10) {
  let cur = { x: pt.x, y: pt.y };
  let pushCount = 0;
  let resolved = true;

  for (let iter = 0; iter < maxIter; iter++) {
    let anyInside = false;
    for (const obs of obstacles) {
      const res = testPoint(cur.x, cur.y, obs);
      if (res.inside) {
        anyInside = true;
        // Push out along the surface normal by (depth + small margin)
        const margin = 0.5;
        const displacement = (res.depth + margin) * pushStrength;
        cur.x += res.normal.x * displacement;
        cur.y += res.normal.y * displacement;
        pushCount++;
      }
    }
    if (!anyInside) break;
    if (iter === maxIter - 1) resolved = false;
  }
  return { pt: cur, pushCount, resolved };
}

/**
 * Full Collision-Aware Chaikin smoothing
 * @param {Array<{x,y}>} controlPoints
 * @param {Array} obstacles
 * @param {number} iterations
 * @param {number} alpha
 * @param {number} pushStrength
 * @param {number} maxPushIter
 * @returns {{ path, collisionPoints, totalPushes, timeUs, resolved }}
 */
function collisionAwareChaikin(controlPoints, obstacles, iterations = 4, alpha = 0.25, pushStrength = 1.0, maxPushIter = 5) {
  const t0 = performance.now();
  let path = copyPath(controlPoints);
  let totalPushes = 0;
  const collisionPoints = [];  // Recorded only on final iteration for accurate visualization

  for (let iter = 0; iter < iterations; iter++) {
    const isLastIter = (iter === iterations - 1);

    // 1. Standard subdivision pass
    path = chaikinPass(path, alpha);

    // 2. Vertex resolution: push any vertex that landed inside an obstacle
    for (let i = 0; i < path.length; i++) {
      if (i === 0 || i === path.length - 1) continue; // anchors are immovable
      const before = { x: path[i].x, y: path[i].y };
      const { pt, pushCount, resolved } = resolveVertex(path[i], obstacles, pushStrength, maxPushIter);
      if (pushCount > 0) {
        if (isLastIter) {
          collisionPoints.push({ x: before.x, y: before.y, px: pt.x, py: pt.y, resolved });
        }
        totalPushes += pushCount;
      }
      path[i] = pt;
    }

    // 3. Edge resolution: detect segments that tunnel through an obstacle
    //    (both endpoints outside but the segment crosses the boundary).
    //    Insert a resolved midpoint for each such crossing.
    const before = path.length;
    path = resolveEdges(path, obstacles, pushStrength, maxPushIter);
    totalPushes += (path.length - before); // count insertions as constraint operations
  }

  const timeUs = (performance.now() - t0) * 1000;
  const allResolved = findCollisions(path, obstacles).length === 0;
  return { path, collisionPoints, totalPushes, timeUs, resolved: allResolved };
}

/* ─── Memory Estimation ──────────────────────────────────────────────────────── */

/**
 * Estimate memory for a path.
 * Each JS {x,y} object ≈ 64 bytes in V8 (object header ~48B + 2×f64 = 16B).
 * Using 64 bytes per point as a realistic lower bound.
 */
function estimateMemoryKB(path) {
  return ((path.length * 64) / 1024).toFixed(2);
}

/* ─── Export ─────────────────────────────────────────────────────────────────── */
window.Algo = {
  standardChaikin,
  collisionAwareChaikin,
  findCollisions,
  testPoint,
  segmentCircle,
  segmentVsRect,
  segmentVsObstacle,
  resolveEdges,
  estimateMemoryKB,
  copyPath,
  lerp2,
  vsub, vadd, vscale, vmag, vnorm, dot,
};
