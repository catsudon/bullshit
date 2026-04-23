/**
 * renderer.js — Canvas 2D rendering engine
 * Handles all drawing: obstacles, paths, waypoints, collision indicators, etc.
 */

'use strict';

window.Renderer = (function () {
  const COLORS = {
    bg:           '#0a0c14',
    grid:         'rgba(255,255,255,0.03)',
    gridAccent:   'rgba(255,255,255,0.07)',
    waypoint:     '#a78bfa',
    waypointFill: 'rgba(167,139,250,0.15)',
    standard:     '#f59e0b',
    standardFill: 'rgba(245,158,11,0.08)',
    aware:        '#10b981',
    awareFill:    'rgba(16,185,129,0.08)',
    obstacle:     'rgba(239,68,68,0.5)',
    obstacleFill: 'rgba(239,68,68,0.10)',
    obstacleShadow:'rgba(239,68,68,0.15)',
    collision:    '#f87171',
    pushVec:      '#60a5fa',
    ghost:        'rgba(255,255,255,0.12)',
  };

  /**
   * Draw background + optional grid
   */
  function drawBackground(ctx, w, h, showGrid) {
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, w, h);

    if (!showGrid) return;
    const step = 40;
    ctx.beginPath();
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += step) {
      ctx.moveTo(x, 0); ctx.lineTo(x, h);
    }
    for (let y = 0; y < h; y += step) {
      ctx.moveTo(0, y); ctx.lineTo(w, y);
    }
    ctx.stroke();

    // Accent lines every 200px
    ctx.beginPath();
    ctx.strokeStyle = COLORS.gridAccent;
    for (let x = 0; x < w; x += 200) {
      ctx.moveTo(x, 0); ctx.lineTo(x, h);
    }
    for (let y = 0; y < h; y += 200) {
      ctx.moveTo(0, y); ctx.lineTo(w, y);
    }
    ctx.stroke();
  }

  /**
   * Draw all obstacles
   */
  function drawObstacles(ctx, obstacles) {
    for (const obs of obstacles) {
      ctx.save();
      if (obs.type === 'circle') {
        // Glow effect
        const grad = ctx.createRadialGradient(obs.x, obs.y, 0, obs.x, obs.y, obs.r);
        grad.addColorStop(0, 'rgba(239,68,68,0.08)');
        grad.addColorStop(1, 'transparent');
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, obs.r + 6, 0, Math.PI * 2);
        ctx.fillStyle = grad;
        ctx.fill();
        // Fill
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, obs.r, 0, Math.PI * 2);
        ctx.fillStyle = COLORS.obstacleFill;
        ctx.fill();
        // Stroke
        ctx.strokeStyle = COLORS.obstacle;
        ctx.lineWidth = 2;
        ctx.stroke();
        // Center dot
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, 3, 0, Math.PI * 2);
        ctx.fillStyle = COLORS.obstacle;
        ctx.fill();
      } else if (obs.type === 'rect') {
        // Shadow glow
        ctx.shadowColor = 'rgba(239,68,68,0.3)';
        ctx.shadowBlur = 12;
        ctx.fillStyle = COLORS.obstacleFill;
        ctx.fillRect(obs.x, obs.y, obs.w, obs.h);
        ctx.shadowBlur = 0;
        ctx.strokeStyle = COLORS.obstacle;
        ctx.lineWidth = 2;
        ctx.strokeRect(obs.x, obs.y, obs.w, obs.h);
        // Corner marks
        const cs = 8;
        ctx.strokeStyle = 'rgba(239,68,68,0.8)';
        ctx.lineWidth = 2;
        [[obs.x, obs.y], [obs.x + obs.w, obs.y], [obs.x, obs.y + obs.h], [obs.x + obs.w, obs.y + obs.h]].forEach(([cx, cy]) => {
          ctx.beginPath(); ctx.arc(cx, cy, cs / 2, 0, Math.PI * 2); ctx.stroke();
        });
      }
      ctx.restore();
    }
  }

  /**
   * Draw a smooth path (array of {x,y}) with optional fill under
   */
  function drawPath(ctx, path, strokeColor, fillColor, lineWidth = 2, showFill = false) {
    if (path.length < 2) return;
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for (let i = 1; i < path.length; i++) {
      ctx.lineTo(path[i].x, path[i].y);
    }
    if (showFill && path.length > 2) {
      ctx.closePath();
    }
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = lineWidth;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.stroke();
    if (showFill && fillColor) {
      ctx.fillStyle = fillColor;
      ctx.fill();
    }
    ctx.restore();
  }

  /**
   * Draw a path with glow effect (2 passes: blurred glow + sharp line)
   */
  function drawGlowPath(ctx, path, color, glowColor, lineWidth = 2.5) {
    if (path.length < 2) return;
    ctx.save();
    // Glow pass
    ctx.shadowColor = glowColor;
    ctx.shadowBlur = 14;
    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for (let i = 1; i < path.length; i++) ctx.lineTo(path[i].x, path[i].y);
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.stroke();
    ctx.shadowBlur = 0;
    // Sharp pass
    ctx.stroke();
    ctx.restore();
  }

  /**
   * Draw control polygon (waypoints connected with dashed line)
   */
  function drawControlPolygon(ctx, waypoints) {
    if (waypoints.length < 2) return;
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(waypoints[0].x, waypoints[0].y);
    for (let i = 1; i < waypoints.length; i++) {
      ctx.lineTo(waypoints[i].x, waypoints[i].y);
    }
    ctx.setLineDash([4, 6]);
    ctx.strokeStyle = 'rgba(167,139,250,0.3)';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
  }

  /**
   * Draw waypoint circles
   */
  function drawWaypoints(ctx, waypoints, selectedIdx = -1) {
    for (let i = 0; i < waypoints.length; i++) {
      const p = waypoints[i];
      const isFirst = i === 0;
      const isLast  = i === waypoints.length - 1;
      const isSelected = i === selectedIdx;

      ctx.save();
      // Outer ring glow
      ctx.beginPath();
      ctx.arc(p.x, p.y, isSelected ? 14 : 10, 0, Math.PI * 2);
      ctx.fillStyle = isSelected ? 'rgba(167,139,250,0.3)' : COLORS.waypointFill;
      ctx.fill();
      // Inner filled circle
      ctx.beginPath();
      ctx.arc(p.x, p.y, isFirst || isLast ? 7 : 5, 0, Math.PI * 2);
      ctx.fillStyle = isFirst ? '#a78bfa' : isLast ? '#c084fc' : COLORS.waypoint;
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5;
      ctx.fill();
      ctx.stroke();
      // Label for first/last
      if (isFirst || isLast) {
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 9px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(isFirst ? 'S' : 'E', p.x, p.y);
      }
      // Index number
      ctx.fillStyle = 'rgba(255,255,255,0.5)';
      ctx.font = '10px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(String(i + 1), p.x, p.y + 9);
      ctx.restore();
    }
  }

  /**
   * Draw collision points (where vertices were pushed)
   */
  function drawCollisionPoints(ctx, collisionPoints) {
    for (const cp of collisionPoints) {
      ctx.save();
      // Original position (ghost)
      ctx.beginPath();
      ctx.arc(cp.x, cp.y, 4, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(248,113,113,0.5)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([2, 2]);
      ctx.stroke();
      ctx.setLineDash([]);
      // Pushed position
      ctx.beginPath();
      ctx.arc(cp.px, cp.py, 4, 0, Math.PI * 2);
      ctx.fillStyle = cp.resolved ? 'rgba(34,197,94,0.7)' : 'rgba(248,113,113,0.7)';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.restore();
    }
  }

  /**
   * Draw push vectors (arrows showing displacement)
   */
  function drawPushVectors(ctx, collisionPoints) {
    for (const cp of collisionPoints) {
      const dx = cp.px - cp.x;
      const dy = cp.py - cp.y;
      const len = Math.sqrt(dx * dx + dy * dy);
      if (len < 1) continue;
      ctx.save();
      ctx.strokeStyle = COLORS.pushVec;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(cp.x, cp.y);
      ctx.lineTo(cp.px, cp.py);
      ctx.stroke();
      // Arrowhead
      const angle = Math.atan2(dy, dx);
      const al = 7;
      ctx.beginPath();
      ctx.moveTo(cp.px, cp.py);
      ctx.lineTo(cp.px - al * Math.cos(angle - 0.4), cp.py - al * Math.sin(angle - 0.4));
      ctx.lineTo(cp.px - al * Math.cos(angle + 0.4), cp.py - al * Math.sin(angle + 0.4));
      ctx.closePath();
      ctx.fillStyle = COLORS.pushVec;
      ctx.fill();
      ctx.restore();
    }
  }

  /**
   * Draw obstacle preview cursor (while dragging to place rect)
   */
  function drawRectPreview(ctx, x, y, w, h) {
    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(239,68,68,0.7)';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(x, y, w, h);
    ctx.fillStyle = 'rgba(239,68,68,0.08)';
    ctx.fillRect(x, y, w, h);
    ctx.setLineDash([]);
    ctx.restore();
  }

  /**
   * Draw the obstacle count badges on canvas (top-left info)
   */
  function drawCanvasInfo(ctx, stdPath, awareResult, obstacles, stdViol, awaViol) {
    const lines = [];
    if (stdPath) lines.push({ label: 'Std vertices:', val: stdPath.length, color: '#f59e0b' });
    if (awareResult) {
      // Use pre-cached violation counts (computed in recompute(), not per-frame)
      lines.push({ label: 'Std violations:', val: stdViol ?? 0, color: (stdViol ?? 0) > 0 ? '#f87171' : '#22c55e' });
      lines.push({ label: 'Aware vertices:', val: awareResult.path.length, color: '#10b981' });
      lines.push({ label: 'Aware violations:', val: awaViol ?? 0, color: '#22c55e' });
    }

    if (lines.length === 0) return;

    const pad = 10, lh = 16, w2 = 150;
    const bh = pad * 2 + lh * lines.length;
    ctx.save();
    ctx.fillStyle = 'rgba(10,12,20,0.8)';
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(10, 10, w2, bh, 6);
    ctx.fill();
    ctx.stroke();
    for (let i = 0; i < lines.length; i++) {
      const l = lines[i];
      ctx.font = '11px Inter, sans-serif';
      ctx.fillStyle = 'rgba(255,255,255,0.4)';
      ctx.textAlign = 'left';
      ctx.fillText(l.label, 20, 10 + pad + i * lh + 4);
      ctx.font = 'bold 11px JetBrains Mono, monospace';
      ctx.fillStyle = l.color;
      ctx.textAlign = 'right';
      ctx.fillText(String(l.val), 10 + w2 - 10, 10 + pad + i * lh + 4);
    }
    ctx.restore();
  }

  /**
   * Main render function
   */
  function render(canvas, state) {
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;

    drawBackground(ctx, W, H, state.showGrid);
    drawObstacles(ctx, state.obstacles);

    // Control polygon
    if (state.showWaypoints && state.waypoints.length > 1) {
      drawControlPolygon(ctx, state.waypoints);
    }

    // Standard Chaikin path
    if (state.showStandard && state.stdPath && state.stdPath.length > 1) {
      drawGlowPath(ctx, state.stdPath, COLORS.standard, 'rgba(245,158,11,0.4)', 2.5);
    }

    // Collision-Aware path
    if (state.showAware && state.awareResult && state.awareResult.path.length > 1) {
      drawGlowPath(ctx, state.awareResult.path, COLORS.aware, 'rgba(16,185,129,0.4)', 2.5);
    }

    // Collision points & push vectors
    if (state.showCollisions && state.awareResult) {
      drawCollisionPoints(ctx, state.awareResult.collisionPoints);
    }
    if (state.showNormals && state.awareResult) {
      drawPushVectors(ctx, state.awareResult.collisionPoints);
    }

    // Waypoints on top
    if (state.showWaypoints) {
      drawWaypoints(ctx, state.waypoints, state.selectedWaypoint);
    }

    // Rect drag preview
    if (state.rectDrag) {
      const { x, y, cx, cy } = state.rectDrag;
      drawRectPreview(ctx, Math.min(x, cx), Math.min(y, cy), Math.abs(cx - x), Math.abs(cy - y));
    }

    // Canvas info badge (reads cached violation counts from state)
    drawCanvasInfo(ctx, state.stdPath, state.awareResult, state.obstacles, state.stdViol, state.awaViol);
  }

  /**
   * Draw the mini benchmark line chart (right panel)
   */
  function drawBenchChart(canvas, data) {
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const pad = { t: 10, r: 10, b: 28, l: 36 };
    const cw = W - pad.l - pad.r, ch = H - pad.t - pad.b;

    ctx.fillStyle = 'transparent';
    ctx.clearRect(0, 0, W, H);

    if (!data || data.ns.length === 0) {
      ctx.fillStyle = 'rgba(255,255,255,0.1)';
      ctx.font = '11px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Run benchmark to see data', W / 2, H / 2);
      return;
    }

    const maxY = Math.max(...data.stdTimes, ...data.awareTimes, 1);
    const minN = Math.min(...data.ns), maxN = Math.max(...data.ns);

    const toX = (n) => pad.l + ((n - minN) / (maxN - minN || 1)) * cw;
    const toY = (t) => pad.t + ch - (t / maxY) * ch;

    // Axes
    ctx.strokeStyle = 'rgba(255,255,255,0.1)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pad.l, pad.t); ctx.lineTo(pad.l, pad.t + ch);
    ctx.lineTo(pad.l + cw, pad.t + ch);
    ctx.stroke();

    // Grid lines + Y labels
    const yTicks = 4;
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.3)';
    ctx.textAlign = 'right';
    for (let i = 0; i <= yTicks; i++) {
      const v = (maxY * i) / yTicks;
      const y = toY(v);
      ctx.fillText(v.toFixed(0) + 'μs', pad.l - 4, y + 3);
      ctx.strokeStyle = 'rgba(255,255,255,0.05)';
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + cw, y); ctx.stroke();
    }

    // X labels
    ctx.fillStyle = 'rgba(255,255,255,0.3)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let i = 0; i < data.ns.length; i += Math.max(1, Math.floor(data.ns.length / 5))) {
      const x = toX(data.ns[i]);
      ctx.fillText(data.ns[i], x, pad.t + ch + 6);
    }

    // Theoretical O(n) reference line
    const refMax = data.awareTimes[data.awareTimes.length - 1] || 1;
    const refK = refMax / (data.ns[data.ns.length - 1] || 1);
    ctx.beginPath();
    ctx.moveTo(toX(minN), toY(refK * minN));
    ctx.lineTo(toX(maxN), toY(refK * maxN));
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    // Standard line
    ctx.beginPath();
    ctx.moveTo(toX(data.ns[0]), toY(data.stdTimes[0]));
    for (let i = 1; i < data.ns.length; i++) {
      ctx.lineTo(toX(data.ns[i]), toY(data.stdTimes[i]));
    }
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.stroke();

    // Aware line
    ctx.beginPath();
    ctx.moveTo(toX(data.ns[0]), toY(data.awareTimes[0]));
    for (let i = 1; i < data.ns.length; i++) {
      ctx.lineTo(toX(data.ns[i]), toY(data.awareTimes[i]));
    }
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.stroke();

    // Dots
    for (let i = 0; i < data.ns.length; i++) {
      ctx.beginPath();
      ctx.arc(toX(data.ns[i]), toY(data.stdTimes[i]), 3, 0, Math.PI * 2);
      ctx.fillStyle = '#f59e0b';
      ctx.fill();
      ctx.beginPath();
      ctx.arc(toX(data.ns[i]), toY(data.awareTimes[i]), 3, 0, Math.PI * 2);
      ctx.fillStyle = '#10b981';
      ctx.fill();
    }
  }

  return { render, drawBenchChart, COLORS };
})();
