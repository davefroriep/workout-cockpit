/* SS2K Calibration Cockpit - pure logic (no DOM). Works in browser (window.CalCore) and node. */
(function (root) {
  'use strict';
  const Core = {};

  // ---------- small stats helpers ----------
  Core.median = a => { if (!a.length) return NaN; const s = [...a].sort((x, y) => x - y), m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };
  Core.mean = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN;
  Core.sd = a => { if (a.length < 2) return 0; const m = Core.mean(a); return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / (a.length - 1)); };
  Core.mad = a => { const m = Core.median(a); return Core.median(a.map(x => Math.abs(x - m))); };
  Core.pct = (a, p) => { if (!a.length) return NaN; const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.max(0, Math.round(p * (s.length - 1))))]; };
  Core.linreg = (xs, ys) => {
    const n = xs.length; if (n < 2) return { slope: NaN, intercept: NaN, r2: NaN };
    const mx = Core.mean(xs), my = Core.mean(ys); let sxx = 0, sxy = 0, syy = 0;
    for (let i = 0; i < n; i++) { sxx += (xs[i] - mx) ** 2; sxy += (xs[i] - mx) * (ys[i] - my); syy += (ys[i] - my) ** 2; }
    const slope = sxx ? sxy / sxx : NaN; return { slope, intercept: my - slope * mx, r2: sxx && syy ? (sxy * sxy) / (sxx * syy) : NaN };
  };
  // piecewise-linear interpolation; xs must be ascending; clamps at the ends
  Core.interp = (xs, ys, x) => {
    if (!xs.length) return NaN; if (x <= xs[0]) return ys[0]; if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
    let lo = 0, hi = xs.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xs[m] <= x) lo = m; else hi = m; }
    const f = (x - xs[lo]) / (xs[hi] - xs[lo] || 1); return ys[lo] + f * (ys[hi] - ys[lo]);
  };
  Core.round = (v, d = 1) => (v === null || v === undefined || !isFinite(v)) ? null : Math.round(v * 10 ** d) / 10 ** d;
  Core.clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  // ---------- parsers ----------
  // FTMS Indoor Bike Data (0x2AD2): cadence (0.5 rpm), resistance, instantaneous power - all from ONE packet
  Core.parseFtms = function (dv) {
    const flags = dv.getUint16(0, true); let o = 2; const r = {};
    if (!(flags & 1)) { r.speed = dv.getUint16(o, true) / 100; o += 2; }
    if (flags & 2) o += 2;
    if (flags & 4) { r.cad = dv.getUint16(o, true) / 2; o += 2; }
    if (flags & 8) o += 2;
    if (flags & 16) o += 3;
    if (flags & 32) { r.res = dv.getInt16(o, true); o += 2; }
    if (flags & 64) { r.w = dv.getInt16(o, true); o += 2; }
    return r;
  };
  Core.parseSS2KLine = function (line) {
    if (!line || line[0] !== '{') return null;
    let d; try { d = JSON.parse(line); } catch (e) { return null; }
    return (d && d.pot !== undefined && d.pos !== undefined) ? d : null;
  };

  // ---------- geometry helpers ----------
  const CLIP_LO = 30, CLIP_HI = 4065; // pot counts treated as "at the rail"
  Core.CLIP_LO = CLIP_LO; Core.CLIP_HI = CLIP_HI;
  Core.isClipped = p => p <= CLIP_LO || p >= CLIP_HI;

  // Firmware's current C7-level formula (measured only up to ~level 36; above that it is an extrapolation)
  Core.posForC7Level = L => Math.round(2047 + (L - 4.3) * 386.4);
  Core.c7LevelForPos = p => Math.min(100, Math.max(3, 4.3 + (p - 2047) / 386.4));
  Core.estWatts = (level, rpm) => {
    const Ls = [3, 5, 8, 13, 19, 24, 30, 36], Ts = [0.36, 0.38, 0.41, 0.53, 0.80, 1.15, 1.74, 2.43];
    let T; if (level <= 3) T = Ts[0]; else if (level >= 36) T = Ts[7] + 0.115 * (Math.min(level, 100) - 36); else T = Core.interp(Ls, Ts, level);
    return rpm <= 15 ? 0 : T * rpm * Math.pow(rpm / 75, 0.4);
  };

  // pot <-> steps map from a one-direction sweep (unclipped points only)
  Core.makePotMap = function (up) {
    const pts = (up || []).filter(p => !Core.isClipped(p.pot));
    if (pts.length < 4) return null;
    const bySteps = [...pts].sort((a, b) => a.pos - b.pos);
    const sign = Core.linreg(bySteps.map(p => p.pos), bySteps.map(p => p.pot)).slope >= 0 ? 1 : -1;
    const byPot = [...pts].sort((a, b) => a.pot - b.pot);
    const px = [], py = []; // dedupe equal pots
    byPot.forEach(p => { if (px.length && p.pot === px[px.length - 1]) { py[py.length - 1] = (py[py.length - 1] + p.pos) / 2; } else { px.push(p.pot); py.push(p.pos); } });
    return {
      sign, minPot: px[0], maxPot: px[px.length - 1], minPos: bySteps[0].pos, maxPos: bySteps[bySteps.length - 1].pos,
      potToPos: adc => Core.interp(px, py, adc),
      posToPot: pos => Core.interp(bySteps.map(p => p.pos), bySteps.map(p => p.pot), pos),
      // local counts-per-step (signed) around a position
      slopeAt(pos, span = 1200) {
        const a = Core.interp(bySteps.map(p => p.pos), bySteps.map(p => p.pot), pos - span / 2);
        const b = Core.interp(bySteps.map(p => p.pos), bySteps.map(p => p.pot), pos + span / 2);
        return (b - a) / span;
      }
    };
  };

  // ---------- analyses ----------
  Core.analyzeHoming = function ({ lights, heavies, expectedTravel = 43944 }) {
    const warn = [];
    const lp = lights.map(x => x.pos), hp = heavies.map(x => x.pos);
    const lpot = lights.map(x => x.pot), hpot = heavies.map(x => x.pot);
    const travel = Core.mean(hp) - Core.mean(lp);
    const clipL = lpot.some(Core.isClipped), clipH = hpot.some(Core.isClipped);
    if (travel < expectedTravel * 0.9) warn.push(`Travel ${Math.round(travel)} steps is well under the expected ~${expectedTravel}: a stall may have triggered early (raise motor current or lower the stall threshold).`);
    if (clipL) warn.push('Pot is at the rail at the LIGHT end (clipped) - the pot map and pot-% positions will be unusable there.');
    if (clipH) warn.push('Pot is at the rail at the HEAVY end (clipped) - the pot map and pot-% positions will be unusable there.');
    const potSpan = Math.abs(Core.mean(hpot) - Core.mean(lpot));
    if (potSpan < 2000) warn.push(`Pot span is only ${Math.round(potSpan)} counts between the two ends - expected 3000+ for the long pot.`);
    const lightRep = lights.length > 1 ? Core.sd(lp.slice(1)) : 0, heavyRep = Core.sd(hp);
    return {
      travel: Math.round(travel), lightPosSd: Core.round(lightRep), heavyPosSd: Core.round(heavyRep),
      lightPot: Math.round(Core.mean(lpot)), heavyPot: Math.round(Core.mean(hpot)), lightPotSd: Core.round(Core.sd(lpot)), heavyPotSd: Core.round(Core.sd(hpot)),
      potSpan: Math.round(potSpan), clipped: clipL || clipH, warnings: warn, ok: !warn.length,
      potMode: !(clipL || clipH) && potSpan >= 2000
    };
  };

  Core.analyzePotMap = function (up, down) {
    const warn = [];
    const unclipped = up.filter(p => !Core.isClipped(p.pot));
    const clippedFrac = 1 - unclipped.length / Math.max(1, up.length);
    const map = Core.makePotMap(up);
    let nonMono = 0, slopes = [];
    if (map) {
      const s = [...unclipped].sort((a, b) => a.pos - b.pos);
      for (let i = 1; i < s.length; i++) { const d = (s[i].pot - s[i - 1].pot) * map.sign; if (d < -3) nonMono++; slopes.push(Math.abs((s[i].pot - s[i - 1].pot) / (s[i].pos - s[i - 1].pos))); }
    }
    const stepsPerCount = slopes.filter(x => x > 0).map(x => 1 / x);
    // hysteresis: horizontal gap of the down trace relative to the up trace at equal pot values
    const gaps = [];
    if (map) down.filter(p => !Core.isClipped(p.pot) && p.pot > map.minPot + 40 && p.pot < map.maxPot - 40).forEach(p => gaps.push(p.pos - map.potToPos(p.pot)));
    const absGaps = gaps.map(Math.abs);
    if (clippedFrac > 0.02) warn.push(`${Math.round(clippedFrac * 100)}% of the travel reads at the pot rail (clipped).`);
    if (nonMono > 2) warn.push(`${nonMono} non-monotonic steps in the pot trace (noise, dirty track, or loose linkage).`);
    return {
      points: up.length, clippedFrac: Core.round(clippedFrac, 3), nonMonotonic: nonMono,
      stepsPerCountBest: Core.round(Math.min(...stepsPerCount, 1e9), 1), stepsPerCountWorst: Core.round(Math.max(...stepsPerCount, 0), 1),
      stepsPerCountMedian: Core.round(Core.median(stepsPerCount), 1),
      hysteresisMedian: Core.round(Core.median(absGaps), 0), hysteresisP90: Core.round(Core.pct(absGaps, 0.9), 0),
      warnings: warn, ok: !warn.length && !!map
    };
  };

  // moves: [{frac,speed,kind:'rev'|'fwd',lost}]
  Core.analyzeLash = function (moves) {
    const groups = {};
    moves.forEach(m => { const k = `${m.kind}|${m.speed}|${m.frac}`; (groups[k] = groups[k] || []).push(m.lost); });
    const table = Object.keys(groups).map(k => { const [kind, speed, frac] = k.split('|'); const v = groups[k]; return { kind, speed: +speed, frac: +frac, mean: Core.round(Core.mean(v), 0), sd: Core.round(Core.sd(v), 0), n: v.length }; });
    const out = { table, verdict: {} };
    ['rev', 'fwd'].forEach(kind => {
      const rows = table.filter(r => r.kind === kind); if (!rows.length) return;
      const speeds = [...new Set(rows.map(r => r.speed))].sort((a, b) => a - b);
      const bySpeed = speeds.map(s => Core.mean(rows.filter(r => r.speed === s).map(r => r.mean)));
      const fracs = [...new Set(rows.map(r => r.frac))];
      const byFrac = fracs.map(f => Core.mean(rows.filter(r => r.frac === f).map(r => r.mean)));
      const noise = Core.mean(rows.map(r => r.sd));
      out.verdict[kind] = {
        meanLost: Core.round(Core.mean(rows.map(r => r.mean)), 0), speedSpread: Core.round(Math.max(...bySpeed) - Math.min(...bySpeed), 0),
        positionSpread: Core.round(Math.max(...byFrac) - Math.min(...byFrac), 0), repeatNoise: Core.round(noise, 0),
        speedMatters: (Math.max(...bySpeed) - Math.min(...bySpeed)) > Math.max(60, 2 * noise)
      };
    });
    return out;
  };

  // C7 vs SS2K timing / ratio. rec: {ss2k:[{t,cad}], c7:[{t,cad,w}]}, events: [{t, l}] (cadence cue changes)
  Core.crossTime = (series, t0, t1, from, to, frac) => {
    const lvl = from + frac * (to - from), up = to > from;
    for (let i = 1; i < series.length; i++) {
      const a = series[i - 1], b = series[i]; if (b.t < t0) continue; if (a.t > t1) break;
      if ((up && a.v < lvl && b.v >= lvl) || (!up && a.v > lvl && b.v <= lvl)) return a.t + (lvl - a.v) / ((b.v - a.v) || 1e-9) * (b.t - a.t);
    }
    return NaN;
  };
  Core.analyzeLag = function (rec, events) {
    const c7c = rec.c7.filter(x => x.cad !== undefined).map(x => ({ t: x.t, v: x.cad }));
    const c7w = rec.c7.filter(x => x.w !== undefined).map(x => ({ t: x.t, v: x.w }));
    const ssc = rec.ss2k.map(x => ({ t: x.t, v: x.cad }));
    const meanIn = (s, a, b) => Core.mean(s.filter(p => p.t >= a && p.t <= b).map(p => p.v));
    const lagW = [], lagS = [], tau63 = [];
    for (let i = 0; i < events.length - 1; i++) {
      const ev = events[i].t, next = events[i + 1].t, prev = i > 0 ? events[i - 1].t : rec.t0;
      const c0 = meanIn(c7c, ev - 5000, ev), c1 = meanIn(c7c, next - 6000, next);
      const w0 = meanIn(c7w, ev - 5000, ev), w1 = meanIn(c7w, next - 6000, next);
      const s0 = meanIn(ssc, ev - 5000, ev), s1 = meanIn(ssc, next - 6000, next);
      if (![c0, c1, w0, w1, s0, s1].every(isFinite) || Math.abs(c1 - c0) < 12) continue;
      const tc = Core.crossTime(c7c, ev, next, c0, c1, 0.5);
      const tw = Core.crossTime(c7w, ev, next, w0, w1, 0.5), tw63 = Core.crossTime(c7w, ev, next, w0, w1, 0.63);
      const ts = Core.crossTime(ssc, ev, next, s0, s1, 0.5);
      if (isFinite(tc) && isFinite(tw)) lagW.push((tw - tc) / 1000);
      if (isFinite(tc) && isFinite(ts)) lagS.push((ts - tc) / 1000);
      if (isFinite(tc) && isFinite(tw63)) tau63.push((tw63 - tc) / 1000);
    }
    // cadence ratio (SS2K / C7) using steady samples only
    const ratios = [];
    c7c.forEach(c => {
      const near = rec.ss2k.reduce((best, s) => Math.abs(s.t - c.t) < Math.abs(best.t - c.t) ? s : best, rec.ss2k[0] || { t: 1e18, cad: NaN });
      if (!near || Math.abs(near.t - c.t) > 600 || c.v < 40 || !(near.cad > 0)) return;
      const win = rec.ss2k.filter(s => Math.abs(s.t - c.t) <= 2000).map(s => s.cad);
      if (win.length > 3 && Core.sd(win) < 1.2) ratios.push(near.cad / c.v);
    });
    const lagWatts = Core.median(lagW);
    return {
      stepsUsed: lagW.length, c7WattsLagSec: Core.round(lagWatts, 2), c7WattsTau63Sec: Core.round(Core.median(tau63), 2), ss2kCadLagSec: Core.round(Core.median(lagS), 2),
      cadRatioSS2KoverC7: Core.round(Core.median(ratios), 4), cadRatioN: ratios.length,
      suggestedSettleSec: isFinite(Core.median(tau63)) ? Math.max(8, Math.ceil(3 * Math.max(Core.median(tau63), 1.5) + 2)) : null
    };
  };

  // Does C7 report watts as a staircase (quantized level) or continuously? rec: creep forward at ~constant cadence
  Core.analyzeStaircase = function (rec) {
    const rows = [];
    rec.c7.forEach(c => {
      if (!(c.cad > 30) || c.w === undefined || c.res === undefined) return;
      const near = rec.ss2k.reduce((best, s) => Math.abs(s.t - c.t) < Math.abs(best.t - c.t) ? s : best, rec.ss2k[0]);
      if (!near || Math.abs(near.t - c.t) > 700) return;
      rows.push({ pot: near.pot, pos: near.pos, res: c.res, norm: c.w / Math.pow(c.cad, 1.4) });
    });
    if (rows.length < 12) return { ok: false, verdict: 'not enough C7 packets', n: rows.length };
    const sign = Core.linreg(rows.map(r => r.pos), rows.map(r => r.norm));
    const levels = [...new Set(rows.map(r => r.res))];
    const within = [];
    levels.forEach(L => { const g = rows.filter(r => r.res === L); if (g.length >= 4) { const sl = Core.linreg(g.map(r => r.pos), g.map(r => r.norm)).slope; if (isFinite(sl)) within.push(Math.abs(sl)); } });
    const overall = Math.abs(sign.slope), w = within.length ? Core.median(within) : NaN;
    const ratio = overall > 0 ? w / overall : NaN;
    return {
      ok: true, n: rows.length, levelsSeen: levels.length, withinOverallRatio: Core.round(ratio, 2),
      verdict: !isFinite(ratio) ? 'inconclusive' : ratio < 0.35 ? 'STAIRCASE: C7 watts jump at whole-level boundaries (smooth them in the fit)' : 'CONTINUOUS: C7 watts vary smoothly between levels'
    };
  };

  Core.plateauStats = function (rec, t0, t1) {
    const ss = rec.ss2k.filter(x => x.t >= t0 && x.t <= t1), c7 = rec.c7.filter(x => x.t >= t0 && x.t <= t1);
    const cadC7 = c7.map(x => x.cad).filter(isFinite), w = c7.map(x => x.w).filter(isFinite), res = c7.map(x => x.res).filter(isFinite);
    const pots = ss.map(x => x.pot), cadS = ss.map(x => x.cad);
    return {
      nC7: c7.length, nSS2K: ss.length,
      c7Cad: Core.median(cadC7), c7CadSd: Core.sd(cadC7), c7W: Core.median(w), c7WMad: Core.mad(w), c7Res: Core.median(res),
      ss2kCad: Core.median(cadS), ss2kCadSd: Core.sd(cadS), pot: Core.median(pots), potSd: Core.sd(pots), pos: Core.median(ss.map(x => x.pos))
    };
  };
  Core.plateauQC = function (s, target, o = {}) {
    const lim = Object.assign({ minC7: 8, cadSd: 1.8, potSd: 4, cadTol: 4, wCv: 0.05 }, o), why = [];
    if (s.nC7 < lim.minC7) why.push(`only ${s.nC7} C7 packets`);
    if (!(s.c7Cad > 0)) why.push('no C7 cadence'); else {
      if (s.c7CadSd > lim.cadSd) why.push(`cadence unsteady (sd ${s.c7CadSd.toFixed(1)})`);
      if (Math.abs(s.c7Cad - target) > lim.cadTol) why.push(`cadence ${s.c7Cad.toFixed(0)} not near ${target}`);
    }
    if (s.potSd > lim.potSd) why.push(`pot moving (sd ${s.potSd.toFixed(1)})`);
    if (s.c7W > 0 && s.c7WMad / s.c7W > lim.wCv) why.push('C7 watts noisy');
    return { pass: why.length === 0, reasons: why };
  };

  Core.analyzeHysteresis = function (items) {
    const rows = items.map(i => {
      const nl = i.light.c7W / Math.pow(i.light.c7Cad, 1.4), nh = i.heavy.c7W / Math.pow(i.heavy.c7Cad, 1.4);
      return { level: i.level, potLight: Core.round(i.light.pot, 0), potHeavy: Core.round(i.heavy.pot, 0), dPot: Core.round(i.heavy.pot - i.light.pot, 0), dWattsPct: Core.round((nh / nl - 1) * 100, 1), dResLevels: Core.round(i.heavy.c7Res - i.light.c7Res, 1) };
    });
    return { rows, maxAbsDWattsPct: Core.round(Math.max(...rows.map(r => Math.abs(r.dWattsPct || 0))), 1) };
  };

  // C7 range map: points [{pos,res,n}] ordered by pos
  Core.analyzeC7Range = function (points) {
    const have = points.filter(p => p.n > 0 && isFinite(p.res));
    if (have.length < 4) return { ok: false, verdict: 'C7 sent too few level readings (it may only report while you pedal)' };
    const maxRes = Math.max(...have.map(p => p.res)), minRes = Math.min(...have.map(p => p.res));
    const atMax = have.filter(p => p.res >= maxRes - 0.5), atMin = have.filter(p => p.res <= minRes + 0.5);
    const mid = have.filter(p => p.res > minRes + 3 && p.res < maxRes - 3);
    const slope = mid.length > 3 ? Core.linreg(mid.map(p => p.pos), mid.map(p => p.res)).slope : NaN;
    return {
      ok: true, minRes, maxRes, floorEndsAtPos: Math.max(...atMin.map(p => p.pos)), ceilingStartsAtPos: Math.min(...atMax.map(p => p.pos)),
      stepsPerLevel: isFinite(slope) && slope > 0 ? Core.round(1 / slope, 1) : null, firmwareAssumesStepsPerLevel: 386.4,
      lastPosWithReading: Math.max(...have.map(p => p.pos))
    };
  };

  Core.fitPowerSurface = (store) => {
    const rpmGrid = [50, 60, 70, 80, 90, 100, 110];
    const fracGrid = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
    const upPoints = [];
    for (const f of fracGrid) {
      const id = 'g' + Math.round(f * 100);
      const t = store?.tests?.[id];
      const rows = t?.summary?.results || t?.raw?.plateaus || [];
      if ((t?.status === 'done' || t?.status === 'review') && rows.length) {
        for (const p of rows) {
          const w = isFinite(p.watts) ? p.watts : p.stats?.c7Watts?.median;
          const cad = isFinite(p.c7Cad) ? p.c7Cad : (p.stats?.c7Cad?.median || p.cad);
          const pot = isFinite(p.pot) ? p.pot : p.stats?.pot?.median;
          const level = isFinite(p.level) ? p.level : p.stats?.c7Res?.median;
          if (isFinite(w) && w > 0 && isFinite(cad) && cad > 20) {
            upPoints.push({ frac: f, cad, watts: w, pot, level, pass: p.pass !== false });
          }
        }
      }
    }
    if (upPoints.length < 3) return { ok: false, count: upPoints.length, message: 'Run at least 2 resistance % buttons in Part C to build the live wattage surface.' };

    // Group by frac and fit log-power curve ln(W) = a + b * ln(RPM) per tested frac
    const byFrac = new Map();
    for (const p of upPoints) {
      if (!byFrac.has(p.frac)) byFrac.set(p.frac, []);
      byFrac.get(p.frac).push(p);
    }

    const testedFracs = [...byFrac.keys()].sort((a, b) => a - b);
    const rowByTestedFrac = new Map();
    const potByTestedFrac = new Map();
    const lvlByTestedFrac = new Map();

    for (const f of testedFracs) {
      const pts = byFrac.get(f).slice().sort((a, b) => a.cad - b.cad);
      potByTestedFrac.set(f, Core.median(pts.map(p => p.pot).filter(isFinite)));
      lvlByTestedFrac.set(f, Core.median(pts.map(p => p.level).filter(isFinite)));
      let a = Math.log(pts[0].watts), b = 1.5;
      if (pts.length >= 2) {
        const lr = Core.linreg(pts.map(p => Math.log(Math.max(30, p.cad))), pts.map(p => Math.log(Math.max(5, p.watts))));
        if (isFinite(lr.slope) && isFinite(lr.intercept)) {
          b = Core.clamp(lr.slope, 0.8, 2.6);
          a = lr.intercept;
        }
      } else {
        a = Math.log(Math.max(5, pts[0].watts)) - b * Math.log(Math.max(30, pts[0].cad));
      }
      const row = rpmGrid.map(rpm => {
        // If we have a plateau within 3 RPM, blend heavily toward actual measurement
        const near = pts.find(p => Math.abs(p.cad - rpm) <= 3.5);
        const modelW = Math.exp(a + b * Math.log(rpm));
        return near ? 0.85 * near.watts + 0.15 * modelW : modelW;
      });
      rowByTestedFrac.set(f, row);
    }

    // Interpolate across all 11 fracGrid rows and enforce monotonicity in both frac and rpm
    const table = [];
    const potLut = [];
    const lvlLut = [];
    const r = store?.range || {};
    for (let i = 0; i < fracGrid.length; i++) {
      const f = fracGrid[i];
      const defaultPot = isFinite(r.potLight) && isFinite(r.potHeavy) ? r.potLight + f * (r.potHeavy - r.potLight) : f * 4095;
      potLut.push(Math.round(testedFracs.length >= 2 ? Core.interp(testedFracs, testedFracs.map(tf => potByTestedFrac.get(tf)), f) : defaultPot));
      lvlLut.push(Core.round(testedFracs.length >= 2 ? Core.interp(testedFracs, testedFracs.map(tf => lvlByTestedFrac.get(tf)), f) : 1 + f * 99, 1));

      const row = rpmGrid.map((rpm, j) => {
        const ys = testedFracs.map(tf => rowByTestedFrac.get(tf)[j]);
        let w = testedFracs.length === 1 ? ys[0] * (0.3 + 0.7 * f / Math.max(0.1, testedFracs[0])) : Core.interp(testedFracs, ys, f);
        if (i > 0) w = Math.max(w, table[i - 1][j]);
        return Core.round(Math.max(5, w), 1);
      });
      for (let j = 1; j < row.length; j++) row[j] = Math.max(row[j], row[j - 1]);
      table.push(row);
    }

    const predictWatts = (frac, rpm) => {
      const fc = Core.clamp(frac, 0, 1) * (fracGrid.length - 1);
      const fi = Math.min(fracGrid.length - 2, Math.max(0, Math.floor(fc)));
      const ft = fc - fi;
      const rc = Core.clamp((rpm - rpmGrid[0]) / (rpmGrid[1] - rpmGrid[0]), 0, rpmGrid.length - 1);
      const ri = Math.min(rpmGrid.length - 2, Math.max(0, Math.floor(rc)));
      const rt = rc - ri;
      const w0 = table[fi][ri] * (1 - rt) + table[fi][ri + 1] * rt;
      const w1 = table[fi + 1][ri] * (1 - rt) + table[fi + 1][ri + 1] * rt;
      return w0 * (1 - ft) + w1 * ft;
    };

    const solveTargetFrac = (targetWatts, rpm) => {
      const wMin = predictWatts(0, rpm), wMax = predictWatts(1, rpm);
      if (targetWatts <= wMin) return { frac: 0, pot: potLut[0], clamped: 'low', expectedWatts: Core.round(wMin, 1) };
      if (targetWatts >= wMax) return { frac: 1, pot: potLut[potLut.length - 1], clamped: 'high', expectedWatts: Core.round(wMax, 1) };
      let lo = 0, hi = 1;
      for (let k = 0; k < 20; k++) {
        const mid = 0.5 * (lo + hi);
        if (predictWatts(mid, rpm) < targetWatts) lo = mid; else hi = mid;
      }
      const frac = 0.5 * (lo + hi);
      const pot = Math.round(Core.interp(fracGrid, potLut, frac));
      return { frac: Core.round(frac, 4), pot, clamped: null, expectedWatts: Core.round(predictWatts(frac, rpm), 1) };
    };

    // Check holdout & down-pass residuals if present
    const evalTest = (testId) => {
      const t = store?.tests?.[testId];
      const rows = t?.summary?.results || t?.raw?.plateaus || [];
      if (!rows.length) return null;
      const errs = [];
      for (const p of rows) {
        const wActual = isFinite(p.watts) ? p.watts : p.stats?.c7Watts?.median;
        const cad = isFinite(p.c7Cad) ? p.c7Cad : (p.stats?.c7Cad?.median || p.cad);
        if (isFinite(wActual) && isFinite(cad)) {
          const wPred = predictWatts(p.frac, cad);
          errs.push({ frac: p.frac, cad: Core.round(cad, 1), actual: Core.round(wActual, 1), predicted: Core.round(wPred, 1), errW: Core.round(wPred - wActual, 1), errPct: Core.round((wPred - wActual) / Math.max(10, wActual) * 100, 1) });
        }
      }
      if (!errs.length) return null;
      return {
        n: errs.length,
        maeW: Core.round(Core.mean(errs.map(e => Math.abs(e.errW))), 1),
        maxAbsW: Core.round(Math.max(...errs.map(e => Math.abs(e.errW))), 1),
        mapePct: Core.round(Core.mean(errs.map(e => Math.abs(e.errPct))), 1),
        rows: errs
      };
    };

    return {
      ok: true,
      testedPositions: testedFracs.length,
      totalPlateaus: upPoints.length,
      rpmGrid,
      fracGrid,
      potLut,
      lvlLut,
      table,
      holdoutEval: evalTest('holdout'),
      downEval: evalTest('down'),
      predictWatts,
      solveTargetFrac
    };
  };

  Core.exportFirmwareHeader = (store, surface) => {
    const r = store?.range || {};
    const lashSummary = store?.tests?.lash?.summary?.table || [];
    const fwdLash = Math.round(Core.median(lashSummary.filter(x => x.kind === 'fwd').map(x => x.mean)) || 402);
    const revLash = Math.round(Core.median(lashSummary.filter(x => x.kind === 'rev').map(x => x.mean)) || 402);
    const lagSec = store?.tests?.lag?.summary?.c7WattsTau63Sec || 2.8;
    const potLut = surface?.potLut || [0, 409, 819, 1228, 1638, 2048, 2457, 2867, 3276, 3686, 4095];
    const table = surface?.table || Array.from({ length: 11 }, () => [30, 45, 65, 90, 120, 155, 195]);

    const rowsCpp = table.map((row, i) => `  { ${row.map(w => w.toFixed(1) + 'f').join(', ')} }${i < table.length - 1 ? ',' : ' '} // ${i * 10}% pot (${potLut[i]} ADC)`).join('\n');
    return `// Auto-generated by SS2K Calibration Cockpit on ${new Date().toISOString()}
#pragma once
#include <stdint.h>

// 1. Mechanical Travel & Potentiometer Safe Endpoints
static constexpr int32_t  CAL_SAFE_MIN_STEPS    = ${r.safeMin ?? 200};
static constexpr int32_t  CAL_SAFE_MAX_STEPS    = ${r.safeMax ?? 39000};
static constexpr uint16_t CAL_POT_LIGHT_ADC     = ${r.potLight ?? potLut[0]};
static constexpr uint16_t CAL_POT_HEAVY_ADC     = ${r.potHeavy ?? potLut[10]};
static constexpr int32_t  CAL_BACKLASH_FWD      = ${fwdLash};
static constexpr int32_t  CAL_BACKLASH_REV      = ${revLash};
static constexpr float    CAL_C7_WATTS_TAU_SEC  = ${Number(lagSec).toFixed(2)}f;

// 2. Non-Linear Potentiometer ADC Lookup (0%, 10%, ..., 100% travel)
static constexpr uint16_t CAL_POT_LUT[11] = { ${potLut.join(', ')} };

// 3. Calibrated 2D Wattage Surface: CAL_WATTS_TABLE[11][7]
// Rows: 0% to 100% magnet travel (step 10%)
// Cols: 50, 60, 70, 80, 90, 100, 110 RPM
static constexpr float CAL_RPM_GRID[7] = { 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f, 110.0f };
static constexpr float CAL_WATTS_TABLE[11][7] = {
${rowsCpp}
};
`;
  };

  root.CalCore = Core; if (typeof module !== 'undefined') module.exports = Core;
})(typeof window !== 'undefined' ? window : globalThis);
