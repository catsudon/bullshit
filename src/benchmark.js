/**
 * benchmark.js — Performance benchmarking module
 * Runs scalability tests on both algorithms and draws result charts.
 */

'use strict';

window.Benchmark = (function () {
  // Access Algo at call-time (not parse-time) to avoid load-order crashes

  /**
   * Generate a random path with N waypoints spread across the canvas
   */
  function generateRandomPath(n, w, h) {
    const pts = [];
    const margin = 60;
    for (let i = 0; i < n; i++) {
      pts.push({
        x: margin + Math.random() * (w - margin * 2),
        y: margin + Math.random() * (h - margin * 2),
      });
    }
    return pts;
  }

  /**
   * Generate some random obstacles
   */
  function generateObstacles(count, w, h) {
    const obs = [];
    for (let i = 0; i < count; i++) {
      if (Math.random() < 0.5) {
        obs.push({ type: 'circle', x: 100 + Math.random() * (w - 200), y: 100 + Math.random() * (h - 200), r: 20 + Math.random() * 40 });
      } else {
        const bw = 30 + Math.random() * 80, bh = 30 + Math.random() * 80;
        obs.push({ type: 'rect', x: 100 + Math.random() * (w - 200), y: 100 + Math.random() * (h - 200), w: bw, h: bh });
      }
    }
    return obs;
  }

  /**
   * Measure average time over `reps` repetitions
   */
  function measureAvg(fn, reps = 5) {
    let total = 0;
    for (let i = 0; i < reps; i++) {
      const t0 = performance.now();
      fn();
      total += (performance.now() - t0) * 1000; // μs
    }
    return total / reps;
  }

  /**
   * Run a full benchmark suite:
   * - varies N from minN to maxN
   * - records times for both standard and collision-aware
   * - calls onProgress(pct, label) while running
   * - calls onComplete(data) when done
   */
  async function run(onProgress, onComplete) {
    const W = 800, H = 600;
    const ns       = [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
    const stdTimes  = [];
    const awareTimes = [];
    const obstacles  = generateObstacles(8, W, H);

    for (let i = 0; i < ns.length; i++) {
      const n = ns[i];
      const pct = Math.round((i / ns.length) * 100);
      onProgress(pct, `Testing N=${n} waypoints...`);

      // Allow UI to breathe
      await new Promise(r => setTimeout(r, 10));

      const path = generateRandomPath(n, W, H);
      const ITERS = 4;

      const stdT = measureAvg(() => window.Algo.standardChaikin(path, ITERS, 0.25), 5);
      const awaT = measureAvg(() => window.Algo.collisionAwareChaikin(path, obstacles, ITERS, 0.25, 1.0, 5), 5);

      stdTimes.push(stdT);
      awareTimes.push(awaT);
    }

    onProgress(100, 'Complete!');
    onComplete({ ns, stdTimes, awareTimes });
  }

  /**
   * Draw detailed benchmark result chart in a canvas element
   */
  function drawResultChart(canvas, data) {
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const pad = { t: 30, r: 20, b: 50, l: 65 };
    const cw = W - pad.l - pad.r, ch = H - pad.t - pad.b;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0f1220';
    ctx.fillRect(0, 0, W, H);

    // Title
    ctx.fillStyle = 'rgba(255,255,255,0.8)';
    ctx.font = 'bold 12px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Runtime Comparison: Standard vs Collision-Aware Chaikin', W / 2, 18);

    const maxY = Math.max(...data.stdTimes, ...data.awareTimes, 1);
    const minN = data.ns[0], maxN = data.ns[data.ns.length - 1];

    const toX = (n) => pad.l + (Math.log10(n) - Math.log10(minN)) / (Math.log10(maxN) - Math.log10(minN)) * cw;
    const toY = (t) => pad.t + ch - Math.min((t / maxY) * ch, ch);

    // Grid
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const y = pad.t + (ch / 5) * i;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + cw, y); ctx.stroke();
      const v = maxY - (maxY / 5) * i;
      ctx.fillStyle = 'rgba(255,255,255,0.35)';
      ctx.font = '10px JetBrains Mono, monospace';
      ctx.textAlign = 'right';
      ctx.fillText(v.toFixed(0) + ' μs', pad.l - 6, y + 4);
    }

    // X Axis labels (log scale)
    ctx.fillStyle = 'rgba(255,255,255,0.35)';
    ctx.textAlign = 'center';
    ctx.font = '10px JetBrains Mono, monospace';
    for (let i = 0; i < data.ns.length; i++) {
      const x = toX(data.ns[i]);
      ctx.fillText(data.ns[i], x, pad.t + ch + 16);
      ctx.strokeStyle = 'rgba(255,255,255,0.04)';
      ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, pad.t + ch); ctx.stroke();
    }
    // X Axis label
    ctx.fillStyle = 'rgba(255,255,255,0.4)';
    ctx.font = '10px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Number of Waypoints (log scale)', W / 2, H - 8);
    // Y Axis label
    ctx.save();
    ctx.translate(14, pad.t + ch / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText('Average Solve Time (μs)', 0, 0);
    ctx.restore();

    // Axes
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(pad.l, pad.t); ctx.lineTo(pad.l, pad.t + ch);
    ctx.lineTo(pad.l + cw, pad.t + ch);
    ctx.stroke();

    // Shaded area under aware
    ctx.beginPath();
    ctx.moveTo(toX(data.ns[0]), toY(data.awareTimes[0]));
    for (let i = 1; i < data.ns.length; i++) ctx.lineTo(toX(data.ns[i]), toY(data.awareTimes[i]));
    ctx.lineTo(toX(data.ns[data.ns.length - 1]), pad.t + ch);
    ctx.lineTo(toX(data.ns[0]), pad.t + ch);
    ctx.closePath();
    ctx.fillStyle = 'rgba(16,185,129,0.08)';
    ctx.fill();

    // Shaded area under std
    ctx.beginPath();
    ctx.moveTo(toX(data.ns[0]), toY(data.stdTimes[0]));
    for (let i = 1; i < data.ns.length; i++) ctx.lineTo(toX(data.ns[i]), toY(data.stdTimes[i]));
    ctx.lineTo(toX(data.ns[data.ns.length - 1]), pad.t + ch);
    ctx.lineTo(toX(data.ns[0]), pad.t + ch);
    ctx.closePath();
    ctx.fillStyle = 'rgba(245,158,11,0.06)';
    ctx.fill();

    // Standard line
    ctx.beginPath();
    ctx.moveTo(toX(data.ns[0]), toY(data.stdTimes[0]));
    for (let i = 1; i < data.ns.length; i++) ctx.lineTo(toX(data.ns[i]), toY(data.stdTimes[i]));
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2.5;
    ctx.lineJoin = 'round';
    ctx.shadowColor = 'rgba(245,158,11,0.4)';
    ctx.shadowBlur = 8;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Aware line
    ctx.beginPath();
    ctx.moveTo(toX(data.ns[0]), toY(data.awareTimes[0]));
    for (let i = 1; i < data.ns.length; i++) ctx.lineTo(toX(data.ns[i]), toY(data.awareTimes[i]));
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2.5;
    ctx.lineJoin = 'round';
    ctx.shadowColor = 'rgba(16,185,129,0.4)';
    ctx.shadowBlur = 8;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Dots
    for (let i = 0; i < data.ns.length; i++) {
      ctx.beginPath();
      ctx.arc(toX(data.ns[i]), toY(data.stdTimes[i]), 4, 0, Math.PI * 2);
      ctx.fillStyle = '#f59e0b'; ctx.fill();
      ctx.beginPath();
      ctx.arc(toX(data.ns[i]), toY(data.awareTimes[i]), 4, 0, Math.PI * 2);
      ctx.fillStyle = '#10b981'; ctx.fill();
    }

    // Legend
    const lx = pad.l + cw - 160, ly = pad.t + 10;
    ctx.fillStyle = 'rgba(10,12,20,0.85)';
    ctx.fillRect(lx - 8, ly - 8, 168, 50);
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.strokeRect(lx - 8, ly - 8, 168, 50);
    ctx.fillStyle = '#f59e0b'; ctx.fillRect(lx, ly, 20, 3);
    ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.font = '11px Inter, sans-serif'; ctx.textAlign = 'left';
    ctx.fillText('Standard Chaikin', lx + 26, ly + 4);
    ctx.fillStyle = '#10b981'; ctx.fillRect(lx, ly + 20, 20, 3);
    ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.fillText('Collision-Aware Chaikin', lx + 26, ly + 24);
  }

  /**
   * Compute summary stats from benchmark data
   */
  function computeSummary(data) {
    const n = data.ns.length;
    const lastStd = data.stdTimes[n - 1];
    const lastAwa = data.awareTimes[n - 1];
    const ratio = lastAwa / Math.max(lastStd, 0.01);
    const maxN = data.ns[n - 1];
    const theoreticalFPS = 1e6 / Math.max(lastAwa, 1); // frames per second if this was the only work

    return {
      maxN,
      stdTimeMax: lastStd.toFixed(1),
      awaTimeMax: lastAwa.toFixed(1),
      overheadRatio: ratio.toFixed(2),
      estimatedFPS: Math.min(theoreticalFPS, 9999).toFixed(0),
    };
  }

  return { run, drawResultChart, computeSummary };
})();
