// Browser front-end for the WebAssembly engine.
//
// This file owns rendering and input only. Every physics decision lives in
// core/ and reaches the page through the embind wrapper, so the browser and
// the native CLI cannot drift apart.

(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const canvas = $("view");
  const ctx = canvas.getContext("2d");

  /** Vehicles are drawn as fixed-size glyphs, not to scale: a 2 m airframe
   *  in a 1500 m box would be sub-pixel. Tactical displays use icons for
   *  the same reason. Obstacles are large enough to draw truthfully. */
  const AGENT_GLYPH_PX = 9;
  const TARGET_GLYPH_PX = 5;
  const TRAIL_LENGTH = 220;

  let sim = null;
  let playing = true;
  let stepsPerFrame = 1;
  let showProbes = true;
  let showTrails = true;
  let trails = new Map();
  let view = { scale: 1, offsetX: 0, offsetY: 0 };

  const css = (name) =>
    getComputedStyle(document.documentElement).getPropertyValue(name).trim();

  // ---- parameter plumbing ------------------------------------------------

  /** Controls that map directly onto an engine parameter. `path` mirrors
   *  the shape configure() expects, so adding a knob is a one-line change. */
  const BINDINGS = [
    { id: "navConstant", path: ["guidance", "navConstant"], fmt: (v) => v.toFixed(1) },
    { id: "agentSpeed", path: ["interceptor", "maxSpeedMps"], fmt: (v) => `${v} m/s` },
    { id: "agentHit", path: ["interceptor", "hitRadiusM"], fmt: (v) => `${v} m` },
    { id: "targetSpeed", path: ["target", "maxSpeedMps"], fmt: (v) => `${v} m/s` },
    { id: "numAgents", path: ["numAgents"], fmt: (v) => String(v) },
    { id: "numTargets", path: ["numTargets"], fmt: (v) => String(v) },
  ];

  // Acceleration is the physical quantity people reason about, but the
  // engine takes a force. Convert through mass at the boundary.
  const ACCEL_BINDINGS = [
    { id: "agentAccel", group: "interceptor", out: "agentAccelOut", derived: "agentDerived" },
    { id: "targetAccel", group: "target", out: "targetAccelOut", derived: "targetDerived" },
  ];

  function currentParams() {
    return sim.params();
  }

  function setNested(target, path, value) {
    let node = target;
    for (let i = 0; i < path.length - 1; i += 1) {
      node[path[i]] = node[path[i]] || {};
      node = node[path[i]];
    }
    node[path[path.length - 1]] = value;
  }

  function readNested(source, path) {
    return path.reduce((node, key) => (node == null ? undefined : node[key]), source);
  }

  /** Build the full parameter object from the current control values. */
  function paramsFromControls() {
    const params = currentParams();
    const next = {
      numAgents: params.numAgents,
      numTargets: params.numTargets,
      minObstacles: params.minObstacles,
      maxObstacles: params.maxObstacles,
      interceptor: { ...params.interceptor },
      target: { ...params.target },
      guidance: { ...params.guidance },
    };

    BINDINGS.forEach(({ id, path }) => setNested(next, path, Number($(id).value)));

    ACCEL_BINDINGS.forEach(({ id, group }) => {
      const accel = Number($(id).value);
      next[group].maxForceN = accel * next[group].massKg;
    });

    next.guidance.law = $("law").value;

    // One control drives both ends of the obstacle count, so the slider
    // reads as "how cluttered is the world" rather than exposing a range.
    const obstacles = Number($("obstacles").value);
    next.minObstacles = obstacles;
    next.maxObstacles = obstacles;

    return next;
  }

  function syncOutputs() {
    BINDINGS.forEach(({ id, fmt }) => {
      const out = $(`${id}Out`);
      if (out) out.textContent = fmt(Number($(id).value));
    });

    ACCEL_BINDINGS.forEach(({ id, out, derived, group }) => {
      const accel = Number($(id).value);
      $(out).textContent = `${accel} m/s² · ${(accel / 9.80665).toFixed(1)} g`;

      const speed = group === "interceptor"
        ? Number($("agentSpeed").value)
        : Number($("targetSpeed").value);
      $(derived).textContent = `turn radius ${(speed * speed / accel).toFixed(0)} m at top speed`;
    });

    $("obstaclesOut").textContent = $("obstacles").value;
    $("speedOut").textContent = `${stepsPerFrame}×`;
    $("navConstantField").hidden = !["pn", "apn"].includes($("law").value);
  }

  function applyParams() {
    sim.configure(paramsFromControls());
    trails = new Map();
    syncOutputs();
  }

  /** Seed the controls from the engine's own defaults, so the page never
   *  carries a second copy of the parameter values. */
  function initControlsFromEngine() {
    const params = currentParams();

    BINDINGS.forEach(({ id, path }) => {
      $(id).value = readNested(params, path);
    });

    ACCEL_BINDINGS.forEach(({ id, group }) => {
      $(id).value = params[group].maxForceN / params[group].massKg;
    });

    $("law").value = params.guidance.law;
    $("obstacles").value = Math.round((params.minObstacles + params.maxObstacles) / 2);

    syncOutputs();
  }

  // ---- rendering ---------------------------------------------------------

  function resize() {
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(rect.width * dpr));
    canvas.height = Math.max(1, Math.round(rect.height * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  /** Fit the world box into the canvas, preserving aspect ratio. */
  function updateView(snapshot) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.width / dpr;
    const h = canvas.height / dpr;
    const scale = Math.min(w / snapshot.worldWidthM, h / snapshot.worldHeightM);
    view = {
      scale,
      offsetX: (w - snapshot.worldWidthM * scale) / 2,
      offsetY: (h - snapshot.worldHeightM * scale) / 2,
    };
  }

  const sx = (x) => view.offsetX + x * view.scale;
  const sy = (y) => view.offsetY + y * view.scale;

  function pushTrail(key, x, y) {
    let points = trails.get(key);
    if (!points) {
      points = [];
      trails.set(key, points);
    }
    points.push(x, y);
    if (points.length > TRAIL_LENGTH * 2) points.splice(0, points.length - TRAIL_LENGTH * 2);
  }

  function drawTrail(key, color) {
    const points = trails.get(key);
    if (!points || points.length < 4) return;
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.35;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(sx(points[0]), sy(points[1]));
    for (let i = 2; i < points.length; i += 2) ctx.lineTo(sx(points[i]), sy(points[i + 1]));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  function drawWorldBox(snapshot) {
    ctx.strokeStyle = css("--grid");
    ctx.lineWidth = 1;
    ctx.strokeRect(
      sx(0),
      sy(0),
      snapshot.worldWidthM * view.scale,
      snapshot.worldHeightM * view.scale,
    );
  }

  function drawObstacles(snapshot) {
    ctx.fillStyle = css("--obstacle");
    ctx.strokeStyle = css("--obstacle-line");
    ctx.lineWidth = 1;
    snapshot.obstacles.forEach((o) => {
      ctx.beginPath();
      ctx.arc(sx(o.x), sy(o.y), Math.max(1, o.radiusM * view.scale), 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    });
  }

  function drawProbe(entity) {
    ctx.strokeStyle = css("--probe");
    ctx.globalAlpha = 0.65;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(
      sx(entity.probeX),
      sy(entity.probeY),
      Math.max(1, entity.probeRadiusM * view.scale),
      0,
      Math.PI * 2,
    );
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  function drawAgent(agent) {
    const x = sx(agent.x);
    const y = sy(agent.y);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(agent.heading);
    ctx.fillStyle = css("--agent");
    ctx.beginPath();
    ctx.moveTo(AGENT_GLYPH_PX, 0);
    ctx.lineTo(-AGENT_GLYPH_PX * 0.8, AGENT_GLYPH_PX * 0.62);
    ctx.lineTo(-AGENT_GLYPH_PX * 0.8, -AGENT_GLYPH_PX * 0.62);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function drawTarget(target) {
    ctx.fillStyle = css("--target");
    ctx.beginPath();
    ctx.arc(sx(target.x), sy(target.y), TARGET_GLYPH_PX, 0, Math.PI * 2);
    ctx.fill();
  }

  function draw(snapshot) {
    updateView(snapshot);
    const dpr = window.devicePixelRatio || 1;
    ctx.clearRect(0, 0, canvas.width / dpr, canvas.height / dpr);

    drawWorldBox(snapshot);
    drawObstacles(snapshot);

    snapshot.targets.forEach((t, i) => {
      pushTrail(`t${i}`, t.x, t.y);
      if (showTrails) drawTrail(`t${i}`, css("--target"));
      if (showProbes) drawProbe(t);
      drawTarget(t);
    });

    snapshot.agents.forEach((a, i) => {
      pushTrail(`a${i}`, a.x, a.y);
      if (showTrails) drawTrail(`a${i}`, css("--agent"));
      if (showProbes) drawProbe(a);
      drawAgent(a);
    });
  }

  // ---- readout -----------------------------------------------------------

  const fmt = (value, digits, unit) =>
    value === null || value === undefined ? "—" : `${value.toFixed(digits)}${unit}`;

  function updateReadout(snapshot) {
    $("statSeed").textContent = snapshot.seed;
    $("statTime").textContent = `${snapshot.elapsedS.toFixed(2)} s`;
    $("statTargets").textContent = snapshot.targets.length;
    $("statIntercepts").textContent = snapshot.intercepts;
    $("statMiss").textContent = fmt(snapshot.minMissDistanceM, 2, " m");
    $("statDeltaV").textContent = fmt(snapshot.deltaVMps, 0, " m/s");

    const lead = snapshot.agents[0];
    const engaged = lead && lead.rangeM !== undefined;
    $("statRange").textContent = engaged ? fmt(lead.rangeM, 0, " m") : "—";
    $("statClosing").textContent = engaged ? fmt(lead.closingSpeedMps, 0, " m/s") : "—";
    $("statLos").textContent = engaged ? fmt(lead.losRateRadS, 4, "") : "—";
  }

  // ---- loop --------------------------------------------------------------

  function frame() {
    if (playing && !sim.done()) sim.stepMany(stepsPerFrame);

    const snapshot = sim.snapshot();
    draw(snapshot);
    updateReadout(snapshot);

    // An episode ends when the field is clear; hold the final frame rather
    // than looping, so the result stays readable.
    if (sim.done() && playing) setPlaying(false);

    requestAnimationFrame(frame);
  }

  function setPlaying(next) {
    playing = next;
    $("playPause").textContent = playing ? "Pause" : "Play";
  }

  // ---- wiring ------------------------------------------------------------

  function wireControls() {
    $("playPause").addEventListener("click", () => {
      if (sim.done()) {
        sim.reset();
        trails = new Map();
      }
      setPlaying(!playing);
    });

    $("stepOnce").addEventListener("click", () => {
      setPlaying(false);
      sim.step();
    });

    $("restart").addEventListener("click", () => {
      sim.reset();
      trails = new Map();
      setPlaying(true);
    });

    $("newSeed").addEventListener("click", () => {
      // Drawn here rather than in C++ so the page stays the only source of
      // nondeterminism; the seed is shown and can be replayed.
      sim.reseed(Math.floor(Math.random() * 2 ** 32));
      trails = new Map();
      setPlaying(true);
    });

    $("speed").addEventListener("input", (event) => {
      stepsPerFrame = Number(event.target.value);
      syncOutputs();
    });

    $("showProbes").addEventListener("change", (e) => {
      showProbes = e.target.checked;
    });
    $("showTrails").addEventListener("change", (e) => {
      showTrails = e.target.checked;
      if (!showTrails) trails = new Map();
    });

    // Live-update the labels while dragging, but only rebuild the world on
    // release -- reconfiguring restarts the engagement, and doing that on
    // every pixel of slider travel is unusable.
    const paramInputs = [
      ...BINDINGS.map((b) => b.id),
      ...ACCEL_BINDINGS.map((b) => b.id),
      "obstacles",
    ];
    paramInputs.forEach((id) => {
      $(id).addEventListener("input", syncOutputs);
      $(id).addEventListener("change", () => {
        applyParams();
        setPlaying(true);
      });
    });

    $("law").addEventListener("change", () => {
      applyParams();
      setPlaying(true);
    });

    window.addEventListener("resize", resize);
  }

  // ---- boot --------------------------------------------------------------

  createInterceptionModule()
    .then((Module) => {
      sim = new Module.Simulation();
      $("loading").hidden = true;
      initControlsFromEngine();
      applyParams();
      wireControls();
      resize();
      requestAnimationFrame(frame);
    })
    .catch((error) => {
      $("loading").textContent = `Failed to load engine: ${error}`;
      console.error(error);
    });
})();
