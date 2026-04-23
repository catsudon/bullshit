"""
3D Goldfish Flocking Simulation  (no GLU dependency)
=====================================================
Phase 1 – Random Walk  : Fish drift randomly for ~4 s
Phase 2 – Transition   : ~3 s smooth blend
Phase 3 – Flocking     : Full 3-D boids

Controls
--------
  Left-drag   : orbit camera
  Scroll      : zoom
  R           : reset phases
  Q / Escape  : quit
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
import random, math
import numpy as np
import ctypes

# ── Config ─────────────────────────────────────────────────────────────────────
WIDTH, HEIGHT       = 1000, 720
NUM_FISH            = 6000
BOUNDS              = 120.0       # half-size of cubic tank
MAX_SPEED           = 2.4
PERCEPTION_R        = 35.0
RANDOM_WALK_DUR     = 4.0         # seconds of pure random walk
TRANSITION_DUR      = 3.0         # seconds of blending
TRAIL_LEN           = 20
BODY_LEN            = 4.0
BODY_RAD            = 1.2
TAIL_LEN            = 2.8

FISH_COLORS = [
    (1.00, 0.55, 0.00),
    (1.00, 0.75, 0.00),
    (1.00, 0.35, 0.20),
    (0.95, 0.85, 0.45),
    (1.00, 0.60, 0.40),
]

# ── Math helpers ───────────────────────────────────────────────────────────────
def norm(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else v.copy()

def limit(v: np.ndarray, mag: float) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n * mag if n > mag else v.copy()

def rand_unit() -> np.ndarray:
    v = np.array([random.gauss(0, 1) for _ in range(3)], dtype=np.float32)
    return norm(v)

# ── Pure-Python perspective & look-at (column-major for OpenGL) ────────────────
def perspective_matrix(fovy_deg, aspect, near, far) -> np.ndarray:
    f = 1.0 / math.tan(math.radians(fovy_deg) / 2.0)
    nf = 1.0 / (near - far)
    m = np.zeros((4, 4), dtype=np.float64)
    m[0, 0] = f / aspect
    m[1, 1] = f
    m[2, 2] = (far + near) * nf
    m[2, 3] = -1.0
    m[3, 2] = 2 * far * near * nf
    return m

def lookat_matrix(eye, center, up) -> np.ndarray:
    f = norm(np.array(center) - np.array(eye))
    r = norm(np.cross(f, np.array(up)))
    u = np.cross(r, f)
    m = np.eye(4, dtype=np.float64)
    m[0, :3] = r
    m[1, :3] = u
    m[2, :3] = -f
    m[3, :3] = [-np.dot(r, eye), -np.dot(u, eye), np.dot(f, eye)]
    return m

def load_matrix(m: np.ndarray):
    """Load a row-major numpy 4x4 into OpenGL (column-major)."""
    glLoadMatrixd(m.T.flatten())

# ── Fish ────────────────────────────────────────────────────────────────────────
class Fish:
    def __init__(self, color):
        self.pos   = np.array([random.uniform(-BOUNDS, BOUNDS) for _ in range(3)], dtype=np.float32)
        self.vel   = rand_unit() * random.uniform(0.8, MAX_SPEED)
        self.acc   = np.zeros(3, dtype=np.float32)
        self.color = color
        self.trail: list[np.ndarray] = []
        # random-walk state
        self._rw_target = rand_unit() * MAX_SPEED * 0.7
        self._rw_timer  = random.uniform(0, 1.5)

    # ── Random walk ────────────────────────────────────────────────────────────
    def random_walk(self, dt: float):
        self._rw_timer -= dt
        if self._rw_timer <= 0:
            self._rw_timer  = random.uniform(0.6, 2.2)
            self._rw_target = rand_unit() * random.uniform(0.8, MAX_SPEED)
        steer = self._rw_target - self.vel
        self.acc += steer * 0.12

    # ── Boids ──────────────────────────────────────────────────────────────────
    def boids(self, neighbors):
        if not neighbors:
            return
        sep = ali = coh = np.zeros(3, dtype=np.float32)
        n = 0
        for o in neighbors:
            d = self.pos - o.pos
            dist = np.linalg.norm(d)
            if 1e-3 < dist < PERCEPTION_R:
                sep = sep + norm(d) / dist
                ali = ali + o.vel
                coh = coh + o.pos
                n  += 1
        if n:
            self.acc = (self.acc
                        + sep          * 0.18
                        + (ali / n - self.vel) * 0.10
                        + (coh / n - self.pos) * 0.06)

    # ── Soft boundary ──────────────────────────────────────────────────────────
    def boundary(self):
        MARGIN = 22.0
        for i in range(3):
            over = self.pos[i] - (BOUNDS - MARGIN)
            if over > 0:
                self.acc[i] -= 0.45 * over / MARGIN
            under = -BOUNDS + MARGIN - self.pos[i]
            if under > 0:
                self.acc[i] += 0.45 * under / MARGIN

    # ── Integrate ──────────────────────────────────────────────────────────────
    def update(self):
        self.trail.append(self.pos.copy())
        if len(self.trail) > TRAIL_LEN:
            self.trail.pop(0)
        self.vel  = limit(self.vel + self.acc, MAX_SPEED)
        self.pos += self.vel
        self.pos  = np.clip(self.pos, -BOUNDS, BOUNDS)
        self.acc  = np.zeros(3, dtype=np.float32)


# ── Custom sphere (no GLU) ─────────────────────────────────────────────────────
def _sphere_verts(radius, stacks, slices):
    """Return triangle-strip vertices for a UV sphere."""
    verts = []
    for i in range(stacks):
        phi0 = math.pi * i       / stacks - math.pi / 2
        phi1 = math.pi * (i + 1) / stacks - math.pi / 2
        for j in range(slices + 1):
            theta = 2 * math.pi * j / slices
            ct, st = math.cos(theta), math.sin(theta)
            verts.append((radius * math.cos(phi0) * ct,
                          radius * math.sin(phi0),
                          radius * math.cos(phi0) * st))
            verts.append((radius * math.cos(phi1) * ct,
                          radius * math.sin(phi1),
                          radius * math.cos(phi1) * st))
    return verts

_SPHERE_HI  = _sphere_verts(BODY_RAD,       8, 10)
_SPHERE_LO  = _sphere_verts(BODY_RAD * 0.7, 7, 9)

def draw_sphere_verts(verts):
    glBegin(GL_TRIANGLE_STRIP)
    for v in verts:
        glVertex3f(*v)
    glEnd()


# ── Draw fish ──────────────────────────────────────────────────────────────────
def draw_fish(fish: Fish):
    spd = np.linalg.norm(fish.vel)
    if spd < 1e-6:
        return

    fwd = fish.vel / spd
    tmp = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(fwd, tmp)) > 0.99:
        tmp = np.array([1.0, 0.0, 0.0])
    right = norm(np.cross(fwd, tmp))
    up    = np.cross(right, fwd)

    # Build 4×4 row-major rotation+translation
    R = np.eye(4, dtype=np.float64)
    R[:3, 0] = fwd
    R[:3, 1] = up
    R[:3, 2] = right
    R[3, :3] = fish.pos
    R[3, 3]  = 1.0

    r, g, b = fish.color

    glPushMatrix()
    glMultMatrixd(R.flatten())          # OpenGL reads column-major = our row-major T

    # Body sphere
    glColor4f(r, g, b, 1.0)
    glPushMatrix()
    glTranslatef(-BODY_LEN * 0.25, 0, 0)
    draw_sphere_verts(_SPHERE_HI)
    glPopMatrix()

    # Snout sphere
    glColor4f(r * 0.9, g * 0.9, b * 0.85, 1.0)
    glPushMatrix()
    glTranslatef(BODY_LEN * 0.55, 0, 0)
    draw_sphere_verts(_SPHERE_LO)
    glPopMatrix()

    # Tail fan
    glBegin(GL_TRIANGLE_FAN)
    glColor4f(r * 0.75, g * 0.75, b * 0.75, 0.85)
    glVertex3f(-BODY_LEN * 0.9, 0, 0)
    steps = 14
    for i in range(steps + 1):
        ang = math.pi * 2 * i / steps
        glColor4f(r * 0.6, g * 0.6, b * 0.6, 0.5)
        glVertex3f(-BODY_LEN * 0.9 - TAIL_LEN,
                   math.sin(ang) * BODY_RAD * 0.85,
                   math.cos(ang) * BODY_RAD * 0.85)
    glEnd()

    # Dorsal fin (thin triangle)
    glBegin(GL_TRIANGLES)
    glColor4f(r, g * 0.8, b * 0.5, 0.7)
    glVertex3f( BODY_LEN * 0.3,  BODY_RAD * 1.4, 0)
    glVertex3f(-BODY_LEN * 0.1,  BODY_RAD,       0)
    glVertex3f(-BODY_LEN * 0.5,  BODY_RAD * 1.0, 0)
    glEnd()

    glPopMatrix()


# ── Draw trail ─────────────────────────────────────────────────────────────────
def draw_trail(fish: Fish):
    n = len(fish.trail)
    if n < 2:
        return
    r, g, b = fish.color
    glBegin(GL_LINE_STRIP)
    for i, pt in enumerate(fish.trail):
        alpha = (i / n) * 0.50
        glColor4f(r, g, b, alpha)
        glVertex3f(*pt)
    glEnd()


# ── Draw wireframe tank ────────────────────────────────────────────────────────
def draw_tank():
    B = int(BOUNDS)
    corners = [
        (-B,-B,-B),(B,-B,-B),(B,B,-B),(-B,B,-B),
        (-B,-B, B),(B,-B, B),(B,B, B),(-B,B, B),
    ]
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]
    glLineWidth(1.1)
    glBegin(GL_LINES)
    for a, b in edges:
        glColor4f(0.20, 0.50, 0.90, 0.15)
        glVertex3f(*corners[a])
        glVertex3f(*corners[b])
    glEnd()
    # floor grid
    step = 20
    glBegin(GL_LINES)
    for i in range(-B, B + step, step):
        glColor4f(0.10, 0.30, 0.65, 0.08)
        glVertex3f(i,  -B, -B);  glVertex3f(i,  -B, B)
        glVertex3f(-B, -B,  i);  glVertex3f(B, -B,  i)
    glEnd()
    glLineWidth(1.0)


# ── Spatial hash ───────────────────────────────────────────────────────────────
def build_hash(fishes, cell):
    g = {}
    for f in fishes:
        k = tuple((f.pos / cell).astype(int))
        g.setdefault(k, []).append(f)
    return g

def query(fish, g, cell):
    cx, cy, cz = (fish.pos / cell).astype(int)
    out = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                out.extend(g.get((cx+dx, cy+dy, cz+dz), []))
    return out


# ── HUD (pygame 2-D) ───────────────────────────────────────────────────────────
_font = None
def draw_hud(surface, clock, phase, blend):
    global _font
    if _font is None:
        _font = pygame.font.SysFont("monospace", 18, bold=True)
    fps = clock.get_fps()
    if phase == 0:
        text, col = f"Phase 1 · Random Walk   {fps:.0f} FPS", (255, 200, 80)
    elif phase == 1:
        text, col = f"Transition → Flocking  {int(blend*100)}%   {fps:.0f} FPS", (120, 220, 255)
    else:
        text, col = f"Phase 2 · Flocking      {fps:.0f} FPS", (80, 255, 160)
    hint, hc = "Drag:rotate  Scroll:zoom  R:reset  Q:quit", (160, 180, 200)

    surface.blit(_font.render(text, True, col),  (14, 10))
    surface.blit(_font.render(hint, True, hc),   (14, 32))


# ── Main ────────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3D Goldfish · Random Walk → Flocking")
    clock = pygame.time.Clock()

    # OpenGL state
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

    # Projection (manual, no GLU)
    proj = perspective_matrix(55, WIDTH / HEIGHT, 0.5, 2000)
    glMatrixMode(GL_PROJECTION)
    load_matrix(proj)
    glMatrixMode(GL_MODELVIEW)

    # Camera
    cam_dist  = 340.0
    cam_yaw   = 30.0
    cam_pitch = 18.0
    drag_pos  = None

    # Fishes
    fishes = [Fish(random.choice(FISH_COLORS)) for _ in range(NUM_FISH)]
    total_time = 0.0
    cell = max(1, int(PERCEPTION_R))

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        total_time += dt

        # Phase
        if total_time < RANDOM_WALK_DUR:
            phase, blend = 0, 0.0
        elif total_time < RANDOM_WALK_DUR + TRANSITION_DUR:
            phase = 1
            blend = (total_time - RANDOM_WALK_DUR) / TRANSITION_DUR
        else:
            phase, blend = 2, 1.0

        # Events
        for ev in pygame.event.get():
            if ev.type == QUIT:
                running = False
            elif ev.type == KEYDOWN:
                if ev.key in (K_q, K_ESCAPE):
                    running = False
                elif ev.key == K_r:
                    total_time = 0.0
                    for f in fishes:
                        f.pos   = np.array([random.uniform(-BOUNDS, BOUNDS) for _ in range(3)], dtype=np.float32)
                        f.vel   = rand_unit() * random.uniform(0.5, MAX_SPEED)
                        f.trail = []
            elif ev.type == MOUSEBUTTONDOWN and ev.button == 1:
                drag_pos = pygame.mouse.get_pos()
            elif ev.type == MOUSEBUTTONUP and ev.button == 1:
                drag_pos = None
            elif ev.type == MOUSEMOTION and drag_pos:
                mx, my = pygame.mouse.get_pos()
                cam_yaw   += (mx - drag_pos[0]) * 0.35
                cam_pitch  = max(-85, min(85, cam_pitch + (my - drag_pos[1]) * 0.35))
                drag_pos   = (mx, my)
            elif ev.type == MOUSEWHEEL:
                cam_dist = max(80, min(700, cam_dist - ev.y * 10))

        # Simulation
        grid = build_hash(fishes, cell)
        for f in fishes:
            rw_acc    = np.zeros(3, dtype=np.float32)
            boid_acc  = np.zeros(3, dtype=np.float32)

            # random-walk force
            if blend < 1.0:
                f.random_walk(dt)
                rw_acc  = f.acc.copy()
                f.acc   = np.zeros(3, dtype=np.float32)

            # boids force
            if blend > 0.0:
                neighbors = query(f, grid, cell)
                f.boids(neighbors)
                boid_acc  = f.acc.copy()
                f.acc     = np.zeros(3, dtype=np.float32)

            f.acc = rw_acc * (1 - blend) + boid_acc * blend
            f.boundary()
            f.update()

        # Render 3-D scene
        glClearColor(0.04, 0.09, 0.18, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Camera look-at (manual)
        pr = math.radians(cam_pitch)
        yr = math.radians(cam_yaw)
        eye = np.array([
            cam_dist * math.cos(pr) * math.sin(yr),
            cam_dist * math.sin(pr),
            cam_dist * math.cos(pr) * math.cos(yr),
        ])
        view = lookat_matrix(eye, [0,0,0], [0,1,0])
        glMatrixMode(GL_MODELVIEW)
        load_matrix(view)

        draw_tank()
        for f in fishes:
            draw_trail(f)
        for f in fishes:
            draw_fish(f)

        pygame.display.flip()

        # 2-D HUD overlay on top of the OpenGL frame
        draw_hud(screen, clock, phase, blend)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()