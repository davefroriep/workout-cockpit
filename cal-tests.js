/* SS2K Calibration Cockpit - test definitions and engine helpers (no DOM).
   Every test is an async function run(ctx). All timing goes through ctx.now()/ctx.sleep() so the same
   code runs against the real bike (wall clock) and the simulator (virtual clock). */
(function (root) {
  'use strict';
  const C = root.CalCore || (typeof require !== 'undefined' ? require('./cal-core.js') : null);

  const K = {
    HOME_MA: 750, HOME_FAST: 1500, HOME_SLOW: 400, BACKOFF: 600, MARGIN: 200, SLIP_SEC: 0.02,
    STEP_SPEED: 1500, SETTLE_MS: 700, GUARD_RPM: 40, GUARD_DROP: 7700, OVERSHOOT: 900, EXPECTED_TRAVEL: 43944
  };
  class AbortError extends Error { constructor() { super('Test aborted'); this.name = 'AbortError'; } }

  // ---------------------------------------------------------------- ctx factory
  function makeCtx(o) {
    const ctx = {
      now: o.now, _sleep: o.sleep, _send: o.send, ui: o.ui, store: o.store || {},
      bike: { ss2k: null, c7: null, nSS2K: 0, nC7: 0, nStall: 0 }, aborted: false, stopFlag: false, recs: new Set(), K
    };
    ctx.feed = (kind, d) => {
      const e = Object.assign({ t: ctx.now() }, d);
      if (kind === 'ss2k') { ctx.bike.ss2k = e; ctx.bike.nSS2K++; if (e.stall) ctx.bike.nStall++; } else { ctx.bike.c7 = e; ctx.bike.nC7++; }
      ctx.recs.forEach(r => r[kind].push(e));
    };
    ctx.check = () => { if (ctx.aborted) throw new AbortError(); };
    ctx.sleep = async ms => { ctx.check(); await ctx._sleep(ms); ctx.check(); };
    ctx.send = c => { ctx.check(); return ctx._send(c); };
    ctx.waitUntil = async (pred, timeoutMs, stepMs = 100) => { const t0 = ctx.now(); while (!pred()) { if (ctx.now() - t0 > timeoutMs) return false; await ctx.sleep(stepMs); } return true; };
    ctx.startRec = () => { const r = { ss2k: [], c7: [], t0: ctx.now(), events: [], mark(l) { r.events.push({ t: ctx.now(), l }); }, stop() { ctx.recs.delete(r); r.t1 = ctx.now(); return r; } }; ctx.recs.add(r); return r; };
    ctx.range = () => { if (!ctx.store.range) throw new Error('Run "Home & range" first.'); return ctx.store.range; };
    ctx.potMap = () => { if (!ctx.store.potmap) throw new Error('Run "Pot map" first.'); return C.makePotMap(ctx.store.potmap.up); };
    return ctx;
  }

  // ---------------------------------------------------------------- motion helpers
  async function waitMove(ctx, target, tSend, timeoutMs) {
    const ok = await ctx.waitUntil(() => { const s = ctx.bike.ss2k; return s && s.t > tSend + 350 && s.run === 0 && Math.abs(s.pos - target) <= 3; }, timeoutMs, 50);
    if (!ok) throw new Error(`Sled did not finish moving (at ${ctx.bike.ss2k && ctx.bike.ss2k.pos}, wanted ${target}). A stall or lost BLE link?`);
  }
  async function moveBy(ctx, steps, speed, settleMs) {
    const s0 = ctx.bike.ss2k; if (!s0) throw new Error('No SS2K telemetry yet - connect the bike first.');
    const sp = speed || K.STEP_SPEED, target = s0.pos + steps, tSend = ctx.now();
    ctx.send('SET_SPEED:' + sp); ctx.send('JOG:' + steps);
    await waitMove(ctx, target, tSend, (Math.abs(steps) / sp + 20) * 1000);
    await ctx.sleep(settleMs === undefined ? K.SETTLE_MS : settleMs);
    return ctx.bike.ss2k.pos;
  }
  async function readPot(ctx, ms) {
    const r = ctx.startRec(); await ctx.sleep(ms || 800); r.stop();
    const pots = r.ss2k.map(x => x.pot); return { pot: Math.round(C.median(pots)), sd: C.round(C.sd(pots), 2), n: pots.length };
  }
  // Always finish the move in the requested direction so the lash flank is the same every time.
  async function gotoSteps(ctx, target, o) {
    o = o || {}; const approach = o.approach || 'light', speed = o.speed || K.STEP_SPEED;
    const r = ctx.store.range, maxPos = r ? r.safeMax + K.MARGIN : 60000;
    target = Math.max(60, Math.min(maxPos, Math.round(target)));
    let cur = ctx.bike.ss2k.pos;
    if (approach === 'light' && target < cur) { const pre = Math.max(60, target - K.OVERSHOOT); if (cur > pre) await moveBy(ctx, pre - cur, speed); cur = ctx.bike.ss2k.pos; }
    if (approach === 'heavy' && target > cur) { const pre = Math.min(maxPos, target + K.OVERSHOOT); if (cur < pre) await moveBy(ctx, pre - cur, speed); cur = ctx.bike.ss2k.pos; }
    if (target !== cur) await moveBy(ctx, target - cur, speed);
    return ctx.bike.ss2k.pos;
  }
  async function gotoPot(ctx, adc, o) {
    o = o || {}; const map = ctx.potMap(), tol = o.tol || 6; let target = map.potToPos(adc);
    await gotoSteps(ctx, target, o);
    let rp = await readPot(ctx, 600);
    for (let i = 0; i < 3 && Math.abs(adc - rp.pot) > tol; i++) {
      const slope = map.slopeAt(ctx.bike.ss2k.pos); if (!isFinite(slope) || Math.abs(slope) < 1e-4) break;
      await gotoSteps(ctx, ctx.bike.ss2k.pos + Math.round((adc - rp.pot) / slope), o); rp = await readPot(ctx, 600);
    }
    return { pos: ctx.bike.ss2k.pos, pot: rp.pot };
  }
  const posSafe = (ctx, frac) => { const r = ctx.range(); return Math.round(r.safeMin + frac * (r.safeMax - r.safeMin)); };
  // Calibration span: safe range, shortened to the part the C7 can actually report (after "C7 range")
  function nodeTarget(ctx, frac) {
    const r = ctx.range(), end = r.spanMax || r.safeMax;
    if (r.potMode && ctx.store.potmap) {
      const map = ctx.potMap(), a0 = map.posToPot(r.safeMin), a1 = map.posToPot(end), adc = a0 + frac * (a1 - a0);
      return { pos: Math.round(map.potToPos(adc)), adc: Math.round(adc) };
    }
    return { pos: Math.round(r.safeMin + frac * (end - r.safeMin)), adc: null };
  }
  async function gotoNode(ctx, frac, approach) {
    const t = nodeTarget(ctx, frac); let pot;
    if (t.adc !== null) { const g = await gotoPot(ctx, t.adc, { approach }); pot = g.pot; }
    else { await gotoSteps(ctx, t.pos, { approach }); pot = (await readPot(ctx, 600)).pot; }
    return { frac, pos: ctx.bike.ss2k.pos, pot, adc: t.adc };
  }

  // ---------------------------------------------------------------- rider helpers
  async function timed(ctx, label, sec, each) {
    const t0 = ctx.now();
    while (ctx.now() - t0 < sec * 1000) {
      ctx.ui.timer(label, Math.max(0, sec - (ctx.now() - t0) / 1000));
      if (each) { const r = each(); if (r) return r; }
      await ctx.sleep(250);
    }
    ctx.ui.timer(label, 0); return null;
  }
  async function reachCadence(ctx, cad, timeoutS) {
    ctx.ui.cadTarget(cad); ctx.ui.beep(660, 0.15); ctx.ui.phase(`Get to ${cad} rpm`);
    const t0 = ctx.now(); let since = null;
    while (ctx.now() - t0 < (timeoutS || 35) * 1000) {
      if (ctx.stopFlag) { ctx.stopFlag = false; return 'toohard'; }
      const s = ctx.bike.ss2k;
      if (s && Math.abs(s.cad - cad) <= 3) { since = since === null ? ctx.now() : since; if (ctx.now() - since >= 3000) return 'ok'; } else since = null;
      ctx.ui.timer('Reach target', Math.max(0, (timeoutS || 35) - (ctx.now() - t0) / 1000));
      await ctx.sleep(200);
    }
    return 'reach';
  }
  function guardWatch(ctx, cad) {
    let lowSince = null; const lim = Math.min(K.GUARD_RPM, cad * 0.6);
    return () => {
      if (ctx.stopFlag) { ctx.stopFlag = false; return 'toohard'; }
      const s = ctx.bike.ss2k; if (!s) return null;
      if (s.cad < lim) { lowSince = lowSince === null ? ctx.now() : lowSince; if (ctx.now() - lowSince > 1500) return 'guard'; } else lowSince = null;
      return null;
    };
  }
  async function fireGuard(ctx) {
    const drop = Math.max(0, Math.min(K.GUARD_DROP, ctx.bike.ss2k.pos - 60));
    ctx.send('SET_SPEED:3000'); ctx.send('JOG:' + (-drop)); ctx.ui.beep(300, 0.5); ctx.store.guardFired = (ctx.store.guardFired || 0) + 1;
    await ctx.waitUntil(() => ctx.bike.ss2k.run === 0, 15000, 100); await ctx.sleep(500);
  }
  // One steady-cadence plateau: reach -> settle -> measure -> QC (one automatic retry)
  async function hold(ctx, o) {
    const cad = o.cad, settle = o.settle === undefined ? 12 : o.settle, measure = o.measure === undefined ? 15 : o.measure;
    const reached = await reachCadence(ctx, cad, o.reachTimeoutS);
    if (reached !== 'ok') return { fail: reached, cad };
    let result = null;
    for (let attempt = 1; attempt <= 2; attempt++) {
      ctx.ui.phase('Hold steady');
      const g1 = await timed(ctx, 'Settle', attempt === 1 ? settle : 5, guardWatch(ctx, cad));
      if (g1) { if (g1 === 'guard') await fireGuard(ctx); return { fail: g1, cad }; }
      ctx.ui.phase('Measuring - keep it steady');
      const rec = ctx.startRec(), t0 = ctx.now();
      const g2 = await timed(ctx, 'Measure', measure, guardWatch(ctx, cad)); rec.stop();
      if (g2) { if (g2 === 'guard') await fireGuard(ctx); return { fail: g2, cad }; }
      const st = C.plateauStats(rec, t0, ctx.now()), qc = C.plateauQC(st, cad);
      result = Object.assign({ targetCad: cad, attempt, pass: qc.pass, reasons: qc.reasons }, st);
      if (qc.pass) break;
    }
    ctx.ui.beep(990, 0.12);
    return result;
  }

  // ---------------------------------------------------------------- A1 home both ways
  async function homeEnd(ctx, dir) {
    const label = dir < 0 ? 'light' : 'heavy';
    const approach = async speed => {
      const n0 = ctx.bike.nStall;
      const thrs = speed >= 1200 ? 190 : 100;
      ctx.send('STALL_THRS:' + thrs);
      ctx.send('SET_SPEED:' + speed); ctx.send('RUN:' + dir);
      let lastPot = ctx.bike.ss2k.pot;
      let lastPos = ctx.bike.ss2k.pos;
      const t0 = Date.now();
      const ok = await ctx.waitUntil(() => {
        if (ctx.bike.nStall > n0) return true;
        const curPos = ctx.bike.ss2k.pos;
        const curPot = ctx.bike.ss2k.pot;
        if (Math.abs(curPot - lastPot) >= 2) {
          lastPot = curPot;
          lastPos = curPos;
        } else if (Math.abs(curPos - lastPos) > 850 && (Date.now() - t0) > 600) {
          console.warn('[cal-tests] Pot motion watchdog tripped! Motor moving but pot stationary.');
          return true;
        }
        return false;
      }, 150000, 100);
      ctx.send('STOP'); await ctx.sleep(500);
      if (!ok) throw new Error(`No stall detected at the ${label} end within 150 s. Check motor current / stall threshold, or the sled is jammed.`);
      return ctx.bike.ss2k.pos;
    };
    await approach(K.HOME_FAST);
    await moveBy(ctx, -dir * K.BACKOFF, 1000);
    const pos = await approach(K.HOME_SLOW);
    const rp = await readPot(ctx, 900);
    return { pos, pot: rp.pot, potSd: rp.sd };
  }
  async function runHome(ctx) {
    await ctx.ui.ask('Before the sled moves', 'Nobody on the bike. Make sure the sled is free to travel. The motor will run at low current until it stalls at each end, three times in each direction. About 5 minutes.', [{ id: 'go', label: 'Start homing' }]);
    ctx.send('AUTO_STOP:1'); ctx.send('STALL_THRS:85'); ctx.send('SET_MA:' + K.HOME_MA); await ctx.sleep(500);
    const lights = [], heavies = [], slipEst = Math.round(K.SLIP_SEC * K.HOME_SLOW);
    for (let i = 0; i < 3; i++) {
      ctx.ui.say(`Homing cycle ${i + 1} of 3`, 'Finding the LIGHT end (stall), then the HEAVY end.'); ctx.ui.progress(i / 3);
      ctx.ui.status('Light end...'); const L = await homeEnd(ctx, -1); ctx.send('ZERO'); await ctx.sleep(500); lights.push({ pos: 0, pot: L.pot });
      ctx.ui.status('Heavy end...'); const H = await homeEnd(ctx, +1); heavies.push({ pos: H.pos - slipEst, pot: H.pot });
    }
    ctx.ui.status('Final light-end check...'); const Lf = await homeEnd(ctx, -1); ctx.send('ZERO'); await ctx.sleep(500); lights.push({ pos: 0, pot: Lf.pot });
    const an = C.analyzeHoming({ lights, heavies, expectedTravel: K.EXPECTED_TRAVEL });
    const safeMin = K.MARGIN, safeMax = an.travel - K.MARGIN;
    ctx.ui.status('Measuring the pot at both safe ends...');
    await gotoSteps(ctx, safeMin, { approach: 'light' }); const pL = await readPot(ctx, 900);
    await moveBy(ctx, safeMax - safeMin, K.STEP_SPEED); const pH = await readPot(ctx, 900);
    await gotoSteps(ctx, Math.round((safeMin + safeMax) / 2), { approach: 'heavy' });
    ctx.store.range = { safeMin, safeMax, travel: an.travel, potLight: pL.pot, potHeavy: pH.pot, potMode: an.potMode, spanMax: safeMax, slipEst, at: Date.now() };
    delete ctx.store.potmap; delete ctx.store.c7range;
    ctx.ui.progress(1);
    return { status: an.ok ? 'done' : 'review', summary: Object.assign({}, an, { safeMin, safeMax, potLightSafe: pL.pot, potHeavySafe: pH.pot }), data: { lights, heavies } };
  }

  // ---------------------------------------------------------------- A2 pot map
  async function runPotMap(ctx) {
    const r = ctx.range(); const S = 600, up = [], down = [];
    await ctx.ui.ask('Pot-to-position map', 'The sled steps from the light end to the heavy end and back, reading the pot at every stop. Nobody pedaling. About 5 minutes.', [{ id: 'go', label: 'Start sweep' }]);
    await gotoSteps(ctx, r.safeMin, { approach: 'light' });
    const n = Math.floor((r.safeMax - r.safeMin) / S);
    for (let i = 0; i <= n; i++) {
      if (i > 0) await moveBy(ctx, S, K.STEP_SPEED, 500);
      const rp = await readPot(ctx, 500); up.push({ pos: ctx.bike.ss2k.pos, pot: rp.pot }); ctx.ui.progress(0.5 * i / n); ctx.ui.status(`Up ${i}/${n}  pot ${rp.pot}`);
    }
    for (let i = n; i >= 0; i--) {
      if (i < n) await moveBy(ctx, -S, K.STEP_SPEED, 500);
      const rp = await readPot(ctx, 500); down.push({ pos: ctx.bike.ss2k.pos, pot: rp.pot }); ctx.ui.progress(0.5 + 0.5 * (n - i) / n); ctx.ui.status(`Down ${n - i}/${n}  pot ${rp.pot}`);
    }
    const an = C.analyzePotMap(up, down);
    if (an.ok) ctx.store.potmap = { up, down, at: Date.now() };
    return { status: an.ok ? 'done' : 'review', summary: an, data: { up, down } };
  }

  // ---------------------------------------------------------------- A3 lost motion vs speed
  async function runLash(ctx) {
    const N = 2000, speeds = [500, 1500, 3000], reps = 3, fracs = [0.25, 0.5, 0.75], moves = [], skipped = [];
    ctx.range();
    await ctx.ui.ask('Lost motion vs speed', 'Automatic. The sled makes forward/reverse moves of 2000 steps at three speeds and the pot shows how much of each reversal was lost to slop. Nobody pedaling. About 8 minutes.', [{ id: 'go', label: 'Start' }]);
    const total = fracs.length * speeds.length * reps; let k = 0;
    for (const frac of fracs) for (const speed of speeds) for (let rep = 0; rep < reps; rep++) {
      ctx.ui.status(`Position ${Math.round(frac * 100)}%  speed ${speed} Hz  rep ${rep + 1}/${reps}`); ctx.ui.progress(k++ / total);
      await gotoSteps(ctx, posSafe(ctx, frac) - N / 2, { approach: 'light', speed: 1000 });
      const p0 = (await readPot(ctx, 500)).pot;
      await moveBy(ctx, +N, speed); const p1 = (await readPot(ctx, 500)).pot;
      const c = (p1 - p0) / N;
      if (Math.abs(c * N) < 40 || C.isClipped(p0) || C.isClipped(p1)) { skipped.push({ frac, speed, why: 'pot flat/clipped here' }); continue; }
      await moveBy(ctx, -N, speed); const p2 = (await readPot(ctx, 500)).pot;
      await moveBy(ctx, +N, speed); const p3 = (await readPot(ctx, 500)).pot;
      moves.push({ frac, speed, kind: 'rev', lost: N - (p1 - p2) / c }, { frac, speed, kind: 'fwd', lost: N - (p3 - p2) / c });
    }
    const an = C.analyzeLash(moves); an.skipped = skipped;
    return { status: moves.length >= 12 ? 'done' : 'review', summary: an, data: { moves } };
  }

  // ---------------------------------------------------------------- A4 sequences
  function analyzeSeq(rows) {
    const key = r => `${r.dir > 0 ? 'fwd' : 'rev'}@${r.speed}`;
    const groups = {}; rows.forEach(r => { const k = key(r); (groups[k] = groups[k] || {})[r.prev] = (groups[k][r.prev] || []).concat(r.lost); });
    const table = Object.keys(groups).map(k => {
      const byPrev = groups[k], means = Object.keys(byPrev).map(p => C.mean(byPrev[p])), all = [].concat(...Object.values(byPrev));
      return { move: k, mean: C.round(C.mean(all), 0), sd: C.round(C.sd(all), 0), historySpread: C.round(Math.max(...means) - Math.min(...means), 0), prevKinds: means.length };
    });
    const maxSpread = Math.max(...table.map(t => t.historySpread), 0);
    return { table, maxHistorySpread: maxSpread, verdict: maxSpread < 60 ? 'Lost motion depends only on the direction/speed of the current move (history does not leak) - a simple per-direction constant is enough.' : 'Lost motion depends on what the previous moves were - a learned lash state is needed.' };
  }
  async function runSeq(ctx) {
    const N = 2000, H = 3000, S = 500, loops = 3;
    const map = ctx.potMap(); ctx.range();
    const seqs = [
      { name: 'Fast fwd, slow rev, slow fwd, fast rev', m: [[1, H], [-1, S], [1, S], [-1, H]] },
      { name: 'Slow, slow, fast, slow', m: [[1, S], [-1, S], [1, H], [-1, S]] },
      { name: 'All fast', m: [[1, H], [-1, H], [1, H], [-1, H]] },
      { name: 'All slow', m: [[1, S], [-1, S], [1, S], [-1, S]] },
      { name: 'Fast, fast, slow, slow', m: [[1, H], [-1, H], [1, S], [-1, S]] }
    ];
    await ctx.ui.ask('Move sequences', 'Automatic. Five fast/slow, forward/reverse patterns, three loops each, to see whether one move\'s slop affects the next. Nobody pedaling. About 5 minutes.', [{ id: 'go', label: 'Start' }]);
    const rows = [], drifts = []; let k = 0;
    for (const sq of seqs) {
      ctx.ui.status(sq.name); ctx.ui.progress(k++ / seqs.length);
      await gotoSteps(ctx, posSafe(ctx, 0.5) - N / 2, { approach: 'light', speed: 1000 });
      const c = map.slopeAt(ctx.bike.ss2k.pos, 2 * N); if (!isFinite(c) || Math.abs(c * N) < 40) { drifts.push({ seq: sq.name, skipped: true }); continue; }
      const potStart = (await readPot(ctx, 500)).pot; let prevPot = potStart, prevKey = 'start';
      for (let loop = 0; loop < loops; loop++) for (const [dir, speed] of sq.m) {
        await moveBy(ctx, dir * N, speed); const p = (await readPot(ctx, 500)).pot;
        rows.push({ seq: sq.name, loop, dir, speed, prev: prevKey, lost: N - (p - prevPot) / (c * dir) }); prevPot = p; prevKey = `${dir > 0 ? 'fwd' : 'rev'}@${speed}`;
      }
      drifts.push({ seq: sq.name, driftSteps: C.round((prevPot - potStart) / c, 0) });
    }
    const an = analyzeSeq(rows); an.drifts = drifts;
    return { status: rows.length >= 30 ? 'done' : 'review', summary: an, data: { rows } };
  }

  // ---------------------------------------------------------------- B0 C7 range map
  async function runC7Range(ctx) {
    const r = ctx.range(), step = 1500, pts = []; let spinning = false;
    await ctx.ui.ask('Where does the C7 stop reporting?', 'The sled creeps from light to heavy while we log the level the C7 console shows. Try it WITHOUT pedaling first - if the C7 stays silent I will ask you to spin lightly (about 35 rpm). Tap "Too hard / stop here" when it gets hard. About 3 minutes.', [{ id: 'go', label: 'Start sweep' }]);
    await gotoSteps(ctx, r.safeMin, { approach: 'light' });
    const n = Math.floor((r.safeMax - r.safeMin) / step);
    for (let i = 0; i <= n; i++) {
      if (i > 0) await moveBy(ctx, step, K.STEP_SPEED, 300);
      const rec = ctx.startRec(); await ctx.sleep(2600); rec.stop();
      const res = rec.c7.map(x => x.res).filter(isFinite), w = rec.c7.map(x => x.w).filter(isFinite);
      pts.push({ pos: ctx.bike.ss2k.pos, res: C.median(res), w: C.median(w), n: res.length });
      ctx.ui.progress(i / n); ctx.ui.status(`pos ${ctx.bike.ss2k.pos}  C7 level ${isFinite(C.median(res)) ? C.median(res) : '-'}  (${res.length} packets)`);
      if (!spinning && i === 4 && pts.every(p => p.n === 0)) {
        spinning = true; ctx.ui.cadTarget(35);
        await ctx.ui.ask('C7 is silent while you are not pedaling', 'Please spin lightly at about 35 rpm and keep going. Tap "Too hard / stop here" when it gets hard.', [{ id: 'go', label: 'I am spinning' }]);
      }
      if (ctx.stopFlag) { ctx.stopFlag = false; break; }
    }
    ctx.ui.cadTarget(null);
    const an = C.analyzeC7Range(pts);
    if (an.ok) {
      const reachedCeil = an.maxRes >= 99, end = reachedCeil ? an.ceilingStartsAtPos : an.lastPosWithReading;
      an.spanMax = Math.max(r.safeMin + 3000, Math.min(r.safeMax, end)); an.ceilingReached = reachedCeil;
      ctx.store.c7range = an; ctx.store.range.spanMax = an.spanMax;
    }
    return { status: an.ok ? 'done' : 'review', summary: an, data: { points: pts } };
  }

  // ---------------------------------------------------------------- B1 cadence step / lag
  async function runLag(ctx) {
    ctx.range(); const L = 20;
    await ctx.ui.ask('C7 timing and cadence agreement', `The sled parks at roughly C7 level ${L} (easy). Then follow the cadence cues: 60, 90, 60, 90, 60 rpm, about 20 seconds each, changing quickly each time. About 3 minutes.`, [{ id: 'go', label: 'Park the sled' }]);
    await gotoSteps(ctx, C.posForC7Level(L), { approach: 'light' });
    await ctx.ui.ask('Start pedaling', 'Spin up to an easy 60 rpm, then tap Start. The cues will take over.', [{ id: 'go', label: 'Start' }]);
    const rec = ctx.startRec(), events = [], cues = [[60, 20], [90, 20], [60, 20], [90, 20], [60, 15]];
    for (let i = 0; i < cues.length; i++) {
      ctx.ui.cadTarget(cues[i][0]); ctx.ui.beep(i % 2 ? 990 : 660, 0.25); events.push({ t: ctx.now(), l: cues[i][0] });
      await timed(ctx, `Cue ${i + 1}/${cues.length}`, cues[i][1]); ctx.ui.progress((i + 1) / cues.length);
    }
    events.push({ t: ctx.now(), l: 'end' }); rec.stop(); ctx.ui.cadTarget(null);
    const an = C.analyzeLag(rec, events);
    return { status: an.stepsUsed >= 2 ? 'done' : 'review', summary: an, data: { ss2k: rec.ss2k, c7: rec.c7, events } };
  }

  // ---------------------------------------------------------------- B2 staircase check
  async function runStairs(ctx) {
    ctx.range(); const a = C.posForC7Level(16), b = C.posForC7Level(24), inc = 60;
    await ctx.ui.ask('Does the C7 report in whole-level steps?', 'You hold a steady 80 rpm while the sled creeps through about 8 C7 levels in small steps. We look for watts that jump at level boundaries. About 3 minutes.', [{ id: 'go', label: 'Park the sled' }]);
    await gotoSteps(ctx, a, { approach: 'light' });
    const reached = await reachCadence(ctx, 80, 60); if (reached !== 'ok') return { status: 'review', summary: { ok: false, verdict: 'could not hold 80 rpm' }, data: {} };
    const rec = ctx.startRec(); const n = Math.floor((b - a) / inc);
    for (let i = 0; i < n; i++) { await moveBy(ctx, inc, 1000, 450); await ctx.sleep(600); ctx.ui.progress(i / n); ctx.ui.status(`creep ${i + 1}/${n}`); if (ctx.stopFlag) { ctx.stopFlag = false; break; } }
    rec.stop(); ctx.ui.cadTarget(null);
    const an = C.analyzeStaircase(rec);
    return { status: an.ok ? 'done' : 'review', summary: an, data: { ss2k: rec.ss2k, c7: rec.c7 } };
  }

  // ---------------------------------------------------------------- B3 hysteresis loop
  async function runHyst(ctx) {
    ctx.range(); ctx.potMap(); const levels = [12, 20, 30], items = [], pl = [];
    await ctx.ui.ask('Same pot position from both sides', 'At three sled positions we measure C7 watts at 80 rpm after arriving from the LIGHT side, then again after arriving from the HEAVY side at the same pot reading. This shows whether the C7 sees the knob or the magnet. About 6 minutes.', [{ id: 'go', label: 'Start' }]);
    for (let i = 0; i < levels.length; i++) {
      const L = levels[i]; ctx.ui.status(`Level ~${L}: arriving from light`);
      await gotoSteps(ctx, C.posForC7Level(L), { approach: 'light' });
      const light = await hold(ctx, { cad: 80, settle: 10, measure: 12 });
      if (light.fail) { if (light.fail === 'toohard') break; continue; }
      ctx.ui.status(`Level ~${L}: arriving from heavy`);
      await gotoSteps(ctx, ctx.bike.ss2k.pos + 2500, { approach: 'light' });
      await gotoPot(ctx, Math.round(light.pot), { approach: 'heavy' });
      const heavy = await hold(ctx, { cad: 80, settle: 10, measure: 12 });
      if (heavy.fail) { if (heavy.fail === 'toohard') break; continue; }
      items.push({ level: L, light, heavy }); ctx.ui.progress((i + 1) / levels.length);
    }
    ctx.ui.cadTarget(null);
    const an = items.length ? C.analyzeHysteresis(items) : { rows: [], note: 'no complete pairs' };
    return { status: items.length === levels.length ? 'done' : 'review', summary: an, data: { items } };
  }

  // ---------------------------------------------------------------- position tests (grid / down-pass / holdouts)
  function ladderFor(frac) { let l = [60, 80, 100]; if (frac > 0.3) l = [50, 60, 80, 100]; if (frac > 0.6) l = [40, 50, 60, 80]; return l; }
  function posTest(def) {
    return Object.assign({}, def, {
      run: async ctx => {
        ctx.range(); const out = [], limits = []; let aborted = false;
        for (let fi = 0; fi < def.fracs.length && !aborted; fi++) {
          const frac = def.fracs[fi], pct = Math.round(frac * 100), t = nodeTarget(ctx, frac);
          const lvl = C.c7LevelForPos(t.pos);
          ctx.ui.say(`Moving to ${pct}%`, `Approaching from the ${def.approach} side...`);
          const node = await gotoNode(ctx, frac, def.approach);
          const ladder = (def.ladderFn || ladderFor)(frac);
          const hint = `About C7 level ${lvl.toFixed(0)}. Rough guess ~${C.estWatts(lvl, 80).toFixed(0)} W at 80 rpm (only measured up to level 36).`;
          const a = await ctx.ui.ask(`Sled parked at ${pct}%`, `${hint}\nCadence cues: ${ladder.join(', ')} rpm, each about 40 seconds. Tap "Too hard / stop here" if you cannot hold a cue - the remaining cues at this position are skipped.`, [{ id: 'go', label: 'Start pedaling' }, { id: 'skip', label: 'Skip this position' }, { id: 'abort', label: 'Stop test' }]);
          if (a === 'abort') { aborted = true; break; } if (a === 'skip') { limits.push({ frac, why: 'skipped by rider' }); continue; }
          for (let ci = 0; ci < ladder.length; ci++) {
            let res = await hold(ctx, { cad: ladder[ci], settle: 12, measure: 15 });
            while (res.fail && res.fail !== 'toohard') {
              const b = await ctx.ui.ask(res.fail === 'guard' ? 'Cadence dropped - sled backed off' : 'Could not reach the cue', 'Try this cadence again, or skip the rest at this position?', [{ id: 'retry', label: 'Try again' }, { id: 'toohard', label: 'Too hard - skip the rest' }, { id: 'abort', label: 'Stop test' }]);
              if (b === 'abort') { aborted = true; break; } if (b === 'toohard') { res = { fail: 'toohard', cad: ladder[ci] }; break; }
              if (res.fail === 'guard') await gotoNode(ctx, frac, def.approach);
              res = await hold(ctx, { cad: ladder[ci], settle: 12, measure: 15 });
            }
            if (aborted) break;
            if (res.fail === 'toohard') { limits.push({ frac, cad: ladder[ci], why: 'rider limit' }); break; }
            out.push(Object.assign({ frac, nodePos: node.pos, nodePot: node.pot }, res));
          }
          ctx.ui.cadTarget(null); ctx.ui.progress((fi + 1) / def.fracs.length);
          if (!aborted && fi < def.fracs.length - 1) ctx.ui.say('Rest', 'Stop pedaling and ease off. The next position loads automatically.');
        }
        if (aborted) return { status: 'aborted', summary: { plateaus: out.length }, data: { plateaus: out, limits } };
        const bad = out.filter(p => !p.pass).length;
        return { status: out.length && !bad ? 'done' : 'review', summary: { plateaus: out.length, failedQC: bad, limits, results: out.map(p => ({ frac: p.frac, cad: p.targetCad, c7Cad: C.round(p.c7Cad, 1), watts: C.round(p.c7W, 0), pot: C.round(p.pot, 0), level: C.round(p.c7Res, 0), pass: p.pass, why: p.reasons })) }, data: { plateaus: out, limits } };
      }
    });
  }

  // ---------------------------------------------------------------- the test list
  const needsBench = ['home', 'potmap'], needsGrid = ['home', 'potmap', 'c7range'];
  const TESTS = [
    { id: 'home', part: 'A', title: 'Home & range', minutes: 6, rider: false, needs: [], blurb: 'Finds both hard stops three times, records pot at each end, and sets the safe range.', steps: ['Nobody on the bike, sled free', 'Motor runs to the light stop, then the heavy stop, x3', 'Pot value recorded at each end; clipping and repeatability checked'], run: runHome },
    { id: 'potmap', part: 'A', title: 'Pot map', minutes: 5, rider: false, needs: ['home'], blurb: 'Sweeps the sled up and down in 600-step stops and logs pot vs steps. Builds the pot lookup and measures hysteresis.', steps: ['Sled steps light to heavy, then back', 'Pot read at every stop', 'Pot nonlinearity and slop (hysteresis) measured'], run: runPotMap },
    { id: 'lash', part: 'A', title: 'Lost motion vs speed', minutes: 8, rider: false, needs: bench(), blurb: 'Forward/reverse moves at 500, 1500 and 3000 Hz at three positions. Shows how much of each reversal is lost, and whether speed changes it.', steps: ['3 positions x 3 speeds x 3 reps, automatic', 'Pot is the truth sensor'], run: runLash },
    { id: 'seq', part: 'A', title: 'Move sequences', minutes: 5, rider: false, needs: bench(), blurb: 'Fast/slow, forward/reverse patterns, looped. Shows whether one move leaves slop behind that changes the next.', steps: ['5 patterns x 3 loops, automatic'], run: runSeq },
    { id: 'c7range', part: 'B', title: 'C7 range', minutes: 4, rider: true, needs: bench(), blurb: 'Creeps the sled light to heavy while logging the C7\'s level, to find where the C7 stops being a usable reference.', steps: ['Try without pedaling; spin lightly only if the C7 is silent', 'Tap "Too hard" at the heavy end'], run: runC7Range },
    { id: 'lag', part: 'B', title: 'C7 timing & cadence', minutes: 4, rider: true, needs: ['home'], blurb: 'Cadence steps 60-90-60-90-60 to measure C7 watts lag and the SS2K-vs-C7 cadence ratio.', steps: ['Easy resistance', 'Follow the cues'], run: runLag },
    { id: 'stairs', part: 'B', title: 'C7 staircase check', minutes: 4, rider: true, needs: ['home'], blurb: 'Steady 80 rpm while the sled creeps through ~8 levels. Do C7 watts jump at whole levels or vary smoothly?', steps: ['Hold 80 rpm for ~75 s'], run: runStairs },
    { id: 'hyst', part: 'B', title: 'Same pot, both sides', minutes: 7, rider: true, needs: bench(), blurb: 'Compare C7 watts at the same pot reading after arriving from the light vs the heavy side.', steps: ['3 positions x 2 approaches x 80 rpm'], run: runHyst }
  ];
  function bench() { return ['home', 'potmap']; }
  const NODES = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
  NODES.forEach(f => TESTS.push(posTest({ id: 'g' + Math.round(f * 100), part: 'C', title: `Position ${Math.round(f * 100)}%`, minutes: 5, rider: true, needs: needsGrid, blurb: `Sled to ${Math.round(f * 100)}% of the calibration span, then steady-cadence plateaus.`, steps: ['Sled parks (from the light side)', 'Hold each cadence cue ~40 s'], fracs: [f], approach: 'light' })));
  TESTS.push(posTest({ id: 'down', part: 'D', title: 'Down-pass', minutes: 7, rider: true, needs: needsGrid, blurb: 'Five positions approached from the HEAVY side at 60 and 80 rpm, to compare with the up pass.', steps: ['80%, 60%, 40%, 20%, 0%'], fracs: [0.8, 0.6, 0.4, 0.2, 0.0], approach: 'heavy', ladderFn: f => f > 0.6 ? [60] : [60, 80] }));
  TESTS.push(posTest({ id: 'holdout', part: 'D', title: 'Holdouts', minutes: 6, rider: true, needs: needsGrid, blurb: 'Positions and cadences that were NOT used in the grid, to check the fitted curve.', steps: ['25%, 55%, 85% at 70 and 90 rpm'], fracs: [0.25, 0.55, 0.85], approach: 'light', ladderFn: f => f > 0.5 ? [60, 70] : [70, 90] }));
  const PARTS = { A: 'A  Bench (no rider)', B: 'B  Know the C7', C: 'C  Grid: one position per test', D: 'D  Verify' };

  const API = { K, TESTS, PARTS, makeCtx, AbortError, helpers: { moveBy, gotoSteps, gotoPot, readPot, hold, nodeTarget, gotoNode, analyzeSeq } };
  root.CalTests = API; if (typeof module !== 'undefined') module.exports = API;
})(typeof window !== 'undefined' ? window : globalThis);
