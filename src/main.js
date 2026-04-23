/**
 * main.js — Application orchestrator
 * Handles: event listeners, state management, render loop, presets, UI updates
 */

'use strict';

(function () {
  /* ─── State ────────────────────────────────────────────────────────────────── */
  const state = {
    waypoints:       [],
    obstacles:       [],
    mode:            'waypoint',
    stdPath:         null,
    awareResult:     null,
    selectedWaypoint:-1,
    dragIndex:       -1,
    rectDrag:        null,
    showStandard:    true,
    showAware:       true,
    showWaypoints:   true,
    showCollisions:  true,
    showNormals:     false,
    showGrid:        false,
    iterations:      4,
    alpha:           0.25,
    pushStrength:    1.0,
    maxPushIter:     5,
  };

  let benchData = null;
  let animFrameId = null;
  let lastFPSTime = performance.now();
  let frameCount = 0;
  let currentFPS = 0;
  let _lastPreset = 'maze'; // track last preset for zoom-reset

  /* ─── Canvas Setup ──────────────────────────────────────────────────────────── */
  const canvas = document.getElementById('main-canvas');
  const container = document.getElementById('canvas-container');

  function resizeCanvas() {
    const rect = container.getBoundingClientRect();
    canvas.width  = rect.width;
    canvas.height = rect.height - 38; // subtract toolbar
    recompute();
  }

  window.addEventListener('resize', resizeCanvas);

  /* ─── Recompute Paths ───────────────────────────────────────────────────────── */
  function recompute() {
    if (state.waypoints.length < 2) {
      state.stdPath    = null;
      state.awareResult = null;
      updateMetrics();
      return;
    }

    const t0 = performance.now();
    const stdResult   = Algo.standardChaikin(state.waypoints, state.iterations, state.alpha);
    const awareResult = Algo.collisionAwareChaikin(
      state.waypoints, state.obstacles,
      state.iterations, state.alpha,
      state.pushStrength, state.maxPushIter
    );
    const wallTime = (performance.now() - t0) * 1000;

    state.stdPath    = stdResult.path;
    state.awareResult = awareResult;
    // Cache violation counts so the renderer doesn't recompute every frame
    state.stdViol  = Algo.findCollisions(stdResult.path, state.obstacles).length;
    state.awaViol  = Algo.findCollisions(awareResult.path, state.obstacles).length;

    updateMetrics(stdResult, awareResult, wallTime);
    updateMiniChart();
  }

  /* ─── UI Metrics Update ─────────────────────────────────────────────────────── */
  function updateMetrics(stdResult, awareResult, wallTime) {
    if (!stdResult) {
      document.getElementById('std-verts').textContent = '0';
      document.getElementById('std-coll').textContent  = '--';
      document.getElementById('std-time').textContent  = '0';
      document.getElementById('awa-verts').textContent = '0';
      document.getElementById('awa-coll').textContent  = '0';
      document.getElementById('awa-time').textContent  = '0';
      document.getElementById('awa-pushes').textContent= '0';
      document.getElementById('vertex-val').textContent = '--';
      document.getElementById('time-val').textContent   = '--';
      document.getElementById('mem-std-val').textContent = '-- KB';
      document.getElementById('mem-awa-val').textContent = '-- KB';
      return;
    }

    // Reuse cached violation counts computed in recompute() — no extra O(n·m) pass here
    const stdViol = state.stdViol ?? 0;
    const awaViol = state.awaViol ?? 0;

    document.getElementById('std-verts').textContent = stdResult.path.length;
    document.getElementById('std-coll').textContent  = stdViol;
    document.getElementById('std-coll').className    = stdViol > 0 ? 'danger-val' : 'success-val';
    document.getElementById('std-time').textContent  = stdResult.timeUs.toFixed(1);

    document.getElementById('awa-verts').textContent  = awareResult.path.length;
    document.getElementById('awa-coll').textContent   = awaViol;
    document.getElementById('awa-coll').className     = awaViol > 0 ? 'danger-val' : 'success-val';
    document.getElementById('awa-time').textContent   = awareResult.timeUs.toFixed(1);
    document.getElementById('awa-pushes').textContent = awareResult.totalPushes;

    document.getElementById('vertex-val').textContent = awareResult.path.length;
    document.getElementById('time-val').textContent   = wallTime.toFixed(0) + ' μs';

    // Memory
    const memStd = Algo.estimateMemoryKB(stdResult.path);
    const memAwa = Algo.estimateMemoryKB(awareResult.path);
    document.getElementById('mem-std-val').textContent = memStd + ' KB';
    document.getElementById('mem-awa-val').textContent = memAwa + ' KB';

    const maxMem = Math.max(parseFloat(memStd), parseFloat(memAwa), 0.1);
    document.getElementById('mem-std').style.width = ((parseFloat(memStd) / maxMem) * 90) + '%';
    document.getElementById('mem-awa').style.width = ((parseFloat(memAwa) / maxMem) * 90) + '%';
  }

  /* ─── Mini Chart Update ─────────────────────────────────────────────────────── */
  function updateMiniChart() {
    const miniCanvas = document.getElementById('bench-chart');
    // Only draw real data; never show fabricated placeholder lines
    if (benchData) {
      Renderer.drawBenchChart(miniCanvas, benchData);
    } else {
      Renderer.drawBenchChart(miniCanvas, null);
    }
  }

  /* ─── Render Loop ───────────────────────────────────────────────────────────── */
  function renderLoop() {
    Renderer.render(canvas, state);

    // FPS counter
    frameCount++;
    const now = performance.now();
    if (now - lastFPSTime >= 500) {
      currentFPS = Math.round(frameCount * 1000 / (now - lastFPSTime));
      frameCount = 0;
      lastFPSTime = now;
      document.getElementById('fps-val').textContent = currentFPS;
    }

    animFrameId = requestAnimationFrame(renderLoop);
  }

  /* ─── Canvas Mouse Events ───────────────────────────────────────────────────── */
  function canvasPos(e) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
    };
  }

  function findNearestWaypoint(px, py, threshold = 14) {
    let best = -1, bestDist = threshold * threshold;
    for (let i = 0; i < state.waypoints.length; i++) {
      const p = state.waypoints[i];
      const d2 = (p.x - px) ** 2 + (p.y - py) ** 2;
      if (d2 < bestDist) { bestDist = d2; best = i; }
    }
    return best;
  }

  canvas.addEventListener('mousedown', (e) => {
    const { x, y } = canvasPos(e);
    if (e.button === 2) {
      // Right-click: remove nearest waypoint or obstacle
      const idx = findNearestWaypoint(x, y, 18);
      if (idx >= 0) {
        state.waypoints.splice(idx, 1);
        recompute();
        return;
      }
      // Remove obstacle
      for (let i = state.obstacles.length - 1; i >= 0; i--) {
        const obs = state.obstacles[i];
        const res = Algo.testPoint(x, y, obs);
        if (res.inside) { state.obstacles.splice(i, 1); recompute(); return; }
      }
      return;
    }
    if (e.button !== 0) return;

    if (state.mode === 'waypoint') {
      state.waypoints.push({ x, y });
      recompute();
    } else if (state.mode === 'drag') {
      const idx = findNearestWaypoint(x, y);
      state.dragIndex = idx;
      state.selectedWaypoint = idx;
    } else if (state.mode === 'obstacle-circle') {
      // Start circle drag
      state.rectDrag = { x, y, cx: x, cy: y, type: 'circle' };
    } else if (state.mode === 'obstacle-rect') {
      state.rectDrag = { x, y, cx: x, cy: y, type: 'rect' };
    }
  });

  canvas.addEventListener('mousemove', (e) => {
    const { x, y } = canvasPos(e);
    // Update obstacle cursor preview
    const cursor = document.getElementById('obstacle-cursor');
    if (state.mode === 'obstacle-circle') {
      cursor.style.display = 'block';
      // Position relative to canvas element (not canvas-container) to avoid toolbar offset
      const canvasRect = canvas.getBoundingClientRect();
      cursor.style.left = (e.clientX - canvasRect.left) + 'px';
      cursor.style.top  = (e.clientY - canvasRect.top)  + 'px';
      const r = state.rectDrag ? Math.hypot(x - state.rectDrag.x, y - state.rectDrag.y) : 30;
      cursor.style.width  = (r * 2) + 'px';
      cursor.style.height = (r * 2) + 'px';
    } else {
      cursor.style.display = 'none';
    }

    if (state.dragIndex >= 0 && state.mode === 'drag') {
      state.waypoints[state.dragIndex] = { x, y };
      recompute();
    }
    if (state.rectDrag) {
      state.rectDrag.cx = x;
      state.rectDrag.cy = y;
    }

    // Update hint
    const idx = findNearestWaypoint(x, y, 18);
    if (idx >= 0 && state.mode === 'drag') {
      document.getElementById('canvas-hint').textContent = `Dragging waypoint ${idx + 1}`;
    }
  });

  canvas.addEventListener('mouseup', (e) => {
    const { x, y } = canvasPos(e);
    if (state.dragIndex >= 0) {
      state.dragIndex = -1;
    }
    if (state.rectDrag) {
      const { x: sx, y: sy, cx, cy, type } = state.rectDrag;
      state.rectDrag = null;
      if (type === 'circle') {
        const r = Math.hypot(cx - sx, cy - sy);
        if (r > 5) {
          state.obstacles.push({ type: 'circle', x: sx, y: sy, r });
          recompute();
        }
      } else if (type === 'rect') {
        const rw = Math.abs(cx - sx), rh = Math.abs(cy - sy);
        if (rw > 5 && rh > 5) {
          state.obstacles.push({ type: 'rect', x: Math.min(sx, cx), y: Math.min(sy, cy), w: rw, h: rh });
          recompute();
        }
      }
    }
  });

  canvas.addEventListener('contextmenu', e => e.preventDefault());

  /* ─── Mode Buttons ──────────────────────────────────────────────────────────── */
  document.querySelectorAll('.mode-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      state.mode = btn.dataset.mode;
      state.selectedWaypoint = -1;
      updateHint();
    });
  });

  function updateHint() {
    const hints = {
      'waypoint':         'Click to add waypoints · Right-click to remove',
      'drag':             'Click & drag waypoints to reposition · Right-click to remove',
      'obstacle-circle':  'Click & drag to place circular obstacle · Right-click to remove',
      'obstacle-rect':    'Click & drag to draw rectangular obstacle · Right-click to remove',
    };
    document.getElementById('canvas-hint').textContent = hints[state.mode] || '';
  }

  /* ─── Sliders ───────────────────────────────────────────────────────────────── */
  document.getElementById('iter-slider').addEventListener('input', (e) => {
    state.iterations = parseInt(e.target.value);
    document.getElementById('iter-label').textContent = state.iterations;
    recompute();
  });
  document.getElementById('ratio-slider').addEventListener('input', (e) => {
    state.alpha = parseFloat(e.target.value);
    document.getElementById('ratio-label').textContent = state.alpha.toFixed(2);
    recompute();
  });
  document.getElementById('push-slider').addEventListener('input', (e) => {
    state.pushStrength = parseFloat(e.target.value);
    document.getElementById('push-label').textContent = state.pushStrength.toFixed(1);
    recompute();
  });
  document.getElementById('maxiter-slider').addEventListener('input', (e) => {
    state.maxPushIter = parseInt(e.target.value);
    document.getElementById('maxiter-label').textContent = state.maxPushIter;
    recompute();
  });

  /* ─── Toggles ───────────────────────────────────────────────────────────────── */
  document.getElementById('show-standard').addEventListener('change', (e) => { state.showStandard  = e.target.checked; });
  document.getElementById('show-aware').addEventListener('change',    (e) => { state.showAware     = e.target.checked; });
  document.getElementById('show-waypoints').addEventListener('change',(e) => { state.showWaypoints = e.target.checked; });
  document.getElementById('show-collisions').addEventListener('change',(e)=> { state.showCollisions= e.target.checked; });
  document.getElementById('show-normals').addEventListener('change',  (e) => { state.showNormals   = e.target.checked; });
  document.getElementById('show-grid').addEventListener('change',     (e) => { state.showGrid      = e.target.checked; });

  /* ─── Presets ───────────────────────────────────────────────────────────────── */
  function loadPreset(name) {
    _lastPreset = name;
    state.waypoints  = [];
    state.obstacles  = [];
    const W = canvas.width, H = canvas.height;
    const cx = W / 2, cy = H / 2;

    if (name === 'maze') {
      // Maze corridor: path through narrow gaps
      state.waypoints = [
        { x: W * 0.05, y: H * 0.5 },
        { x: W * 0.25, y: H * 0.5 },
        { x: W * 0.5,  y: H * 0.3 },
        { x: W * 0.75, y: H * 0.5 },
        { x: W * 0.95, y: H * 0.5 },
      ];
      // Walls with narrow gaps
      state.obstacles = [
        { type: 'rect', x: W*0.3-8,  y: 0,       w: 16, h: H*0.4 },
        { type: 'rect', x: W*0.3-8,  y: H*0.55,  w: 16, h: H*0.45 },
        { type: 'rect', x: W*0.6-8,  y: 0,       w: 16, h: H*0.45 },
        { type: 'rect', x: W*0.6-8,  y: H*0.55,  w: 16, h: H },
        { type: 'circle', x: W*0.45, y: H*0.5, r: 30 },
      ];
    } else if (name === 'room') {
      // Room with scattered pillars
      state.waypoints = [
        { x: W * 0.08, y: H * 0.08 },
        { x: W * 0.3,  y: H * 0.5 },
        { x: W * 0.6,  y: H * 0.3 },
        { x: W * 0.8,  y: H * 0.7 },
        { x: W * 0.95, y: H * 0.92 },
      ];
      state.obstacles = [
        { type: 'circle', x: W*0.25, y: H*0.25, r: 28 },
        { type: 'circle', x: W*0.5,  y: H*0.55, r: 35 },
        { type: 'circle', x: W*0.75, y: H*0.35, r: 25 },
        { type: 'rect',   x: W*0.4,  y: H*0.1, w: 60, h: 60 },
        { type: 'rect',   x: W*0.65, y: H*0.6, w: 55, h: 55 },
      ];
    } else if (name === 'narrow') {
      // Narrow gap challenge
      const gapY = H * 0.5, gapSize = 60;
      state.waypoints = [
        { x: W * 0.05, y: H * 0.5 },
        { x: cx, y: H * 0.5 },
        { x: W * 0.95, y: H * 0.5 },
      ];
      state.obstacles = [
        { type: 'rect', x: cx - 6, y: 0,           w: 12, h: gapY - gapSize / 2 },
        { type: 'rect', x: cx - 6, y: gapY + gapSize / 2, w: 12, h: H },
        { type: 'circle', x: W*0.25, y: H*0.5, r: 40 },
        { type: 'circle', x: W*0.75, y: H*0.5, r: 40 },
      ];
    } else if (name === 'stress') {
      // Stress test: many waypoints + many obstacles
      const n = 12;
      for (let i = 0; i < n; i++) {
        const t = i / (n - 1);
        const x = W * 0.05 + t * W * 0.9;
        const y = cy + Math.sin(t * Math.PI * 3) * H * 0.3;
        state.waypoints.push({ x, y });
      }
      for (let i = 0; i < 10; i++) {
        if (i % 2 === 0) {
          state.obstacles.push({ type: 'circle', x: W * (0.1 + i * 0.09), y: cy + (i % 3 - 1) * 60, r: 30 + i * 3 });
        } else {
          state.obstacles.push({ type: 'rect',   x: W * (0.1 + i * 0.09) - 20, y: cy - 50 + (i % 2) * 40, w: 40, h: 40 });
        }
      }
      state.iterations = 6;
      document.getElementById('iter-slider').value = 6;
      document.getElementById('iter-label').textContent = '6';
    }

    recompute();
  }

  document.getElementById('preset-maze').addEventListener('click',   () => loadPreset('maze'));
  document.getElementById('preset-room').addEventListener('click',   () => loadPreset('room'));
  document.getElementById('preset-narrow').addEventListener('click', () => loadPreset('narrow'));
  document.getElementById('preset-stress').addEventListener('click', () => loadPreset('stress'));

  /* ─── Action Buttons ────────────────────────────────────────────────────────── */
  document.getElementById('btn-clear-all').addEventListener('click', () => {
    state.waypoints = []; state.obstacles = [];
    state.stdPath = null; state.awareResult = null;
    recompute();
  });
  document.getElementById('btn-clear-obstacles').addEventListener('click', () => {
    state.obstacles = []; recompute();
  });
  document.getElementById('btn-clear-path').addEventListener('click', () => {
    state.waypoints = [];
    state.stdPath = null; state.awareResult = null;
    recompute();
  });

  /* ─── Benchmark ─────────────────────────────────────────────────────────────── */
  document.getElementById('btn-benchmark').addEventListener('click', () => {
    const modal = document.getElementById('benchmark-modal');
    modal.hidden = false;
    const barEl  = document.getElementById('bench-bar');
    const statEl = document.getElementById('bench-status');
    const chartEl = document.getElementById('bench-result-chart');
    const summaryEl = document.getElementById('bench-summary');
    const summaryGrid = document.getElementById('summary-grid');

    summaryEl.hidden = true;
    barEl.style.width = '0%';
    statEl.textContent = 'Initializing...';

    Benchmark.run(
      (pct, label) => {
        barEl.style.width = pct + '%';
        statEl.textContent = label;
      },
      (data) => {
        benchData = data;
        Benchmark.drawResultChart(chartEl, data);
        const summary = Benchmark.computeSummary(data);
        summaryGrid.innerHTML = `
          <div class="summary-item">
            <div class="summary-item-label">Max N tested</div>
            <div class="summary-item-value">${summary.maxN}</div>
          </div>
          <div class="summary-item">
            <div class="summary-item-label">Overhead ratio</div>
            <div class="summary-item-value" style="color:#f59e0b">${summary.overheadRatio}×</div>
          </div>
          <div class="summary-item">
            <div class="summary-item-label">Std time (N=${summary.maxN})</div>
            <div class="summary-item-value" style="color:#f59e0b">${summary.stdTimeMax} μs</div>
          </div>
          <div class="summary-item">
            <div class="summary-item-label">Aware time (N=${summary.maxN})</div>
            <div class="summary-item-value">${summary.awaTimeMax} μs</div>
          </div>
        `;
        summaryEl.hidden = false;
        updateMiniChart();
      }
    );
  });

  document.getElementById('modal-close').addEventListener('click', () => {
    document.getElementById('benchmark-modal').hidden = true;
  });

  /* ─── Zoom (placeholder — canvas is fixed to container) ──────────────────── */
  document.getElementById('btn-zoom-in').addEventListener('click', () => {
    // Scale waypoints toward center
    const cx = canvas.width / 2, cy = canvas.height / 2;
    const s = 1.1;
    state.waypoints = state.waypoints.map(p => ({
      x: cx + (p.x - cx) * s, y: cy + (p.y - cy) * s,
    }));
    state.obstacles = state.obstacles.map(obs => {
      if (obs.type === 'circle') return { ...obs, x: cx + (obs.x - cx) * s, y: cy + (obs.y - cy) * s, r: obs.r * s };
      return { ...obs, x: cx + (obs.x - cx) * s, y: cy + (obs.y - cy) * s, w: obs.w * s, h: obs.h * s };
    });
    recompute();
  });
  document.getElementById('btn-zoom-out').addEventListener('click', () => {
    const cx = canvas.width / 2, cy = canvas.height / 2;
    const s = 1 / 1.1;
    state.waypoints = state.waypoints.map(p => ({
      x: cx + (p.x - cx) * s, y: cy + (p.y - cy) * s,
    }));
    state.obstacles = state.obstacles.map(obs => {
      if (obs.type === 'circle') return { ...obs, x: cx + (obs.x - cx) * s, y: cy + (obs.y - cy) * s, r: obs.r * s };
      return { ...obs, x: cx + (obs.x - cx) * s, y: cy + (obs.y - cy) * s, w: obs.w * s, h: obs.h * s };
    });
    recompute();
  });
  document.getElementById('btn-zoom-reset').addEventListener('click', () => {
    // Reload the current preset if active, otherwise just refit to canvas
    loadPreset(_lastPreset || 'maze');
  });

  /* ─── Keyboard Shortcuts ────────────────────────────────────────────────────── */
  document.addEventListener('keydown', (e) => {
    if (e.key === 'w' || e.key === 'W') setMode('waypoint');
    if (e.key === 'd' || e.key === 'D') setMode('drag');
    if (e.key === 'c' || e.key === 'C') setMode('obstacle-circle');
    if (e.key === 'r' || e.key === 'R') setMode('obstacle-rect');
    if (e.key === 'Escape') {
      state.dragIndex = -1; state.rectDrag = null;
      document.getElementById('benchmark-modal').hidden = true;
    }
    if (e.key === 'Delete' && state.selectedWaypoint >= 0) {
      state.waypoints.splice(state.selectedWaypoint, 1);
      state.selectedWaypoint = -1;
      recompute();
    }
  });

  function setMode(mode) {
    state.mode = mode;
    document.querySelectorAll('.mode-btn').forEach(b => {
      b.classList.toggle('active', b.dataset.mode === mode);
    });
    updateHint();
  }

  /* ─── Init ──────────────────────────────────────────────────────────────────── */
  function init() {
    resizeCanvas();
    // Start with the maze preset for a dramatic first impression
    setTimeout(() => { loadPreset('maze'); }, 100);
    renderLoop();
    updateHint();
  }

  init();
})();
