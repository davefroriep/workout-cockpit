/* SS2K Calibration Cockpit - virtual bike for Demo mode and headless tests.
   Models: stepper counter vs physical sled (hard stops, counter keeps counting while blocked),
   knob->magnet lash, nonlinear pot, a rider whose cadence follows a cue, and a C7 console that
   reports 1 Hz packets with smoothed watts and integer levels (so a staircase is visible). */
(function (root) {
  'use strict';
  const C = root.CalCore || (typeof require !== 'undefined' ? require('./cal-core.js') : null);

  function mulberry(a) { return function () { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }

  class SimBike {
    constructor(opts) {
      this.o = Object.assign({ lash: 450, flop: 120, travel: 49640, potNoise: 2.5, potSign: 1, c7Tau: 3.0, c7IdleReports: true, riderMaxW: Infinity, seed: 7, ratio: 1.01, stallDetectPackets: 3 }, opts || {});
      this.rnd = mulberry(this.o.seed);
      this.t = 0;
      this.phys = 21000; this.cnt = 500; this.zero = 0;   // counter is arbitrary until homed
      this.mag = this.phys;
      this.v = 0; this.tgt = this.cnt; this.runDir = 0; this.speed = 1500; this.accel = 4000; this.autoStop = true;
      this.blocked = false; this.stallCount = 0; this.stalled = false;
      this.riderTarget = 0; this.cad = 0; this.c7cadS = 0; this.c7wS = 0;
      this._tTel = 0; this._tC7 = 0;
      this.onSS2K = null; this.onC7 = null; this.log = [];
    }
    get pos() { return Math.round(this.cnt - this.zero); }
    setRider(rpm) { this.riderTarget = rpm || 0; }
    level() { return C.c7LevelForPos(this.phys); }
    potOf(m) {
      const x = Math.min(1, Math.max(0, m / this.o.travel)), g = x + 0.15 * Math.sin(Math.PI * x);
      const base = this.o.potSign > 0 ? 100 + 3800 * g : 3900 - 3800 * g;
      return Math.round(Math.min(4095, Math.max(0, base + (this.rnd() - 0.5) * 2 * this.o.potNoise * 1.7)));
    }
    send(cmd) {
      cmd = String(cmd).trim(); this.log.push([this.t, cmd]);
      if (cmd.startsWith('JOG:')) { this.runDir = 0; this.tgt += parseInt(cmd.slice(4), 10) || 0; }
      else if (cmd.startsWith('RUN:')) { this.runDir = Math.sign(parseInt(cmd.slice(4), 10) || 0); if (!this.runDir) this.tgt = this.cnt; }
      else if (cmd === 'STOP') { this.runDir = 0; this.tgt = this.cnt; this.v = 0; }
      else if (cmd.startsWith('SET_SPEED:')) this.speed = Math.min(6000, Math.max(200, parseInt(cmd.slice(10), 10) || 1500));
      else if (cmd === 'ZERO') this.zero = this.cnt;
      else if (cmd.startsWith('AUTO_STOP:')) this.autoStop = !!parseInt(cmd.slice(10), 10);
    }
    _riderEff() {
      const tgt = this.riderTarget; if (!(tgt > 0)) return 0;
      if (!isFinite(this.o.riderMaxW)) return tgt;
      const lvl = this.level(); if (C.estWatts(lvl, tgt) <= this.o.riderMaxW) return tgt;
      let lo = 15, hi = tgt; for (let i = 0; i < 20; i++) { const m = (lo + hi) / 2; if (C.estWatts(lvl, m) > this.o.riderMaxW) hi = m; else lo = m; }
      return lo < 25 ? 0 : lo;
    }
    _step(dt) {
      // --- stepper counter ---
      let vdes;
      if (this.runDir) vdes = this.runDir * this.speed;
      else { const d = this.tgt - this.cnt; vdes = Math.abs(d) < 0.5 ? 0 : Math.sign(d) * Math.min(this.speed, Math.sqrt(2 * this.accel * Math.abs(d))); }
      const dv = vdes - this.v, maxdv = this.accel * dt; this.v += Math.abs(dv) <= maxdv ? dv : Math.sign(dv) * maxdv;
      let dc = this.v * dt;
      if (!this.runDir && Math.abs(this.tgt - this.cnt) <= Math.abs(dc)) { dc = this.tgt - this.cnt; if (Math.abs(vdes) < 1) this.v = 0; }
      this.cnt += dc;
      // --- physical sled with hard stops (counter keeps counting while blocked) ---
      const np = this.phys + dc, cl = Math.min(this.o.travel, Math.max(0, np));
      this.blocked = Math.abs(dc) > 1e-9 && Math.abs(cl - np) > 1e-9; this.phys = cl;
      // --- knob->magnet lash (extra slop at speed) ---
      const hl = this.o.lash / 2 + this.o.flop * Math.min(1, Math.abs(this.v) / 3000) / 2;
      if (this.phys > this.mag + hl) this.mag = this.phys - hl; else if (this.phys < this.mag - hl) this.mag = this.phys + hl;
      // --- rider / C7 ---
      const eff = this._riderEff(), wob = 0.8 * Math.sin(this.t / 2300) + 0.5 * (this.rnd() - 0.5);
      this.cad += (eff - this.cad) * Math.min(1, dt / 1.5); if (eff === 0 && this.cad < 1) this.cad = 0;
      const cadNow = Math.max(0, this.cad + (this.cad > 5 ? wob : 0));
      this.c7cadS += (cadNow - this.c7cadS) * Math.min(1, dt / 0.8);
      this.c7wS += (C.estWatts(this.level(), cadNow) - this.c7wS) * Math.min(1, dt / this.o.c7Tau);
      this._cadNow = cadNow;
    }
    advance(ms) {
      let rem = ms;
      while (rem > 0) {
        const d = Math.min(10, rem); this._step(d / 1000); this.t += d; rem -= d;
        if (this.t - this._tTel >= 200) { this._tTel = this.t; this._telemetry(); }
        if (this.t - this._tC7 >= 1000) { this._tC7 = this.t; this._c7packet(); }
      }
    }
    _telemetry() {
      const running = this.runDir !== 0 || Math.abs(this.tgt - this.cnt) > 0.5 || Math.abs(this.v) > 1;
      const sg = this.blocked ? 20 : 180;
      if (running && sg < 85) this.stallCount++; else this.stallCount = 0;
      this.stalled = this.stallCount >= this.o.stallDetectPackets;
      if (this.stalled && this.autoStop) { this.runDir = 0; this.tgt = this.cnt; this.v = 0; }
      const d = {
        pos: this.pos, pot: this.potOf(this.mag), v: 3.3, sg, ma: 650, spd: this.speed, run: running ? 1 : 0,
        cad: Math.round(this._cadNow * this.o.ratio * 10) / 10, stall: this.stalled ? 1 : 0
      };
      if (this.onSS2K) this.onSS2K(d);
    }
    _c7packet() {
      if (!(this.c7cadS > 3) && !this.o.c7IdleReports) return;
      const d = { cad: Math.round(this.c7cadS * 2) / 2, w: Math.round(this.c7wS), res: Math.round(this.level()) };
      if (this.onC7) this.onC7(d);
    }
  }
  root.SimBike = SimBike; if (typeof module !== 'undefined') module.exports = SimBike;
})(typeof window !== 'undefined' ? window : globalThis);
