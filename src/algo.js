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
 * Returns { hit, point, normal, t } where t in [0,1] is the closest parametric point.
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
  // Check if segment passes through
  if (t1 <= 1 && t2 >= 0) {
    const t = clamp((t1 + t2) / 2, 0, 1);
    const px = ax + dx * t, py = ay + dy * t;
    const nm = vnorm({ x: px - obs.x, y: py - obs.y });
    return { hit: true, t, px, py, normal: nm };
  }
  return { hit: false };
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
  const collisionPoints = [];  // Points that were pushed

  for (let iter = 0; iter < iterations; iter++) {
    // 1. Standard subdivision pass
    path = chaikinPass(path, alpha);

    // 2. Resolve collisions for each vertex
    for (let i = 0; i < path.length; i++) {
      // Don't move the first/last waypoints (anchors)
      if ((i === 0 || i === path.length - 1) && iter === iterations - 1) continue;
      const { pt, pushCount, resolved } = resolveVertex(path[i], obstacles, pushStrength, maxPushIter);
      if (pushCount > 0) {
        collisionPoints.push({ x: path[i].x, y: path[i].y, px: pt.x, py: pt.y, resolved });
        totalPushes += pushCount;
      }
      path[i] = pt;
    }
  }

  const timeUs = (performance.now() - t0) * 1000;
  const allResolved = findCollisions(path, obstacles).length === 0;
  return { path, collisionPoints, totalPushes, timeUs, resolved: allResolved };
}

/* ─── Memory Estimation ──────────────────────────────────────────────────────── */

/**
 * Estimate memory for a path (each {x,y} = 2 doubles = 16 bytes)
 */
function estimateMemoryKB(path) {
  return ((path.length * 2 * 8) / 1024).toFixed(2);
}

/* ─── Export ─────────────────────────────────────────────────────────────────── */
window.Algo = {
  standardChaikin,
  collisionAwareChaikin,
  findCollisions,
  testPoint,
  segmentCircle,
  estimateMemoryKB,
  copyPath,
  lerp2,
  vsub, vadd, vscale, vmag, vnorm, dot,
};
