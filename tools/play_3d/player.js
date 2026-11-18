(() => {
  "use strict";

  const byId = id => document.getElementById(id);
  const ui = {
    scene: byId("scene"), coverage: byId("coverage"), tracks: byId("tracks"),
    play: byId("play"), restart: byId("restart"), resetView: byId("reset-view"),
    speed: byId("speed"), position: byId("position"), clock: byId("clock"),
    segment: byId("segment"), validity: byId("validity"), frame: byId("frame"),
    coordinates: byId("coordinates"), file: byId("file"), format: byId("format"),
    importStatus: byId("import-status"),
  };
  const colors = getComputedStyle(document.documentElement);
  const color = name => colors.getPropertyValue("--" + name).trim();
  const palette = {
    grid: color("grid"), muted: color("muted"), border: color("border"),
    path: color("trajectory"), preview: color("preview"), text: color("text"),
  };
  const viewDefaults = { yaw: -Math.PI / 5, pitch: Math.PI / 5, zoom: 1 };
  const view = { ...viewDefaults };
  const tracks = new Map();
  let track;
  let time = 0;
  let speed = 1;
  let animation = null;
  let startWall = 0;
  let startTime = 0;
  let pointer = null;
  const clamp = (x, min, max) => Math.min(max, Math.max(min, x));

  function showStatus(message, error = false) {
    ui.importStatus.textContent = message;
    ui.importStatus.dataset.error = String(error);
    ui.importStatus.hidden = !message;
  }

  function buildTrack(parsed) {
    const segments = new Map();
    for (const [offset, frame] of parsed.frames.entries()) {
      if (!frame.p) continue;
      let segment = segments.get(frame.segment);
      if (!segment) {
        segment = {
          id: frame.segment, frames: [], min: [...frame.p], max: [...frame.p],
          first: offset, last: offset,
        };
        segments.set(frame.segment, segment);
      }
      segment.frames.push(frame);
      segment.last = offset;
      frame.p.forEach((value, axis) => {
        segment.min[axis] = Math.min(segment.min[axis], value);
        segment.max[axis] = Math.max(segment.max[axis], value);
      });
    }
    const runs = [...segments.values()];
    for (const segment of runs) {
      segment.center = segment.min.map((value, axis) => (value + segment.max[axis]) / 2);
      segment.extent = Math.max(0.025, ...segment.min.map((value, axis) => segment.max[axis] - value));
    }
    const duration = parsed.frames.at(-1).t;
    const cadence = parsed.nominalInterval || (duration / Math.max(1, parsed.frames.length - 1)) || 1 / 60;
    return {
      ...parsed, segments, runs, duration, cadence,
      maxGap: Math.max(0.1, cadence * 5),
      longest: runs.reduce((best, run) =>
        !best || run.frames.length > best.frames.length ? run : best, null),
    };
  }

  function selectTrack(id) {
    pause();
    track = tracks.get(id);
    time = 0;
    ui.position.max = String(Math.max(track.duration, 0.001));
    ui.position.disabled = ui.play.disabled = ui.restart.disabled = track.duration === 0;
    resetView();
    drawCoverage();
  }

  function containingFrame(seconds) {
    let low = 0;
    let high = track.frames.length;
    while (low < high) {
      const middle = (low + high) >>> 1;
      if (track.frames[middle].t <= seconds + 1e-9) low = middle + 1;
      else high = middle;
    }
    return Math.max(0, low - 1);
  }

  // A valid observation occupies its frame interval. A timestamp gap only
  // gets the nominal duration of its last observed frame.
  function frameEnd(index) {
    const current = track.frames[index];
    const next = track.frames[index + 1];
    if (!next) return track.duration;
    return next.t - current.t > track.maxGap
      ? Math.min(next.t, current.t + track.cadence)
      : next.t;
  }

  function fallbackSegment(index) {
    for (let i = track.runs.length - 1; i >= 0; --i) {
      if (track.runs[i].first <= index) return track.runs[i];
    }
    return track.longest;
  }

  function prepareCanvas(canvas) {
    const { width, height } = canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(width * ratio));
    const h = Math.max(1, Math.round(height * ratio));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.lineCap = "butt";
    ctx.lineJoin = "miter";
    return { ctx, width, height };
  }

  function drawCoverage() {
    const { ctx, width, height } = prepareCanvas(ui.coverage);
    const thickness = Math.max(4, height / 3.5);
    const y = (height - thickness) / 2;
    ctx.fillStyle = palette.border;
    ctx.fillRect(0, y, width, thickness);
    if (!track || !track.duration) return;
    ctx.fillStyle = palette.path;
    for (const run of track.runs) {
      const start = track.frames[run.first].t / track.duration * width;
      const end = frameEnd(run.last) / track.duration * width;
      ctx.fillRect(start, y, Math.max(1, end - start), thickness);
    }
  }

  function makeProjection({ yaw, pitch }, origin, center, scale) {
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const cosP = Math.cos(pitch), sinP = Math.sin(pitch);
    return ([px, py, pz]) => {
      const x = px - center[0], y = py - center[1], z = pz - center[2];
      return [
        origin[0] + (cosY * x - sinY * y) * scale,
        origin[1] + ((sinY * x + cosY * y) * sinP - z * cosP) * scale,
      ];
    };
  }

  function line(ctx, a, b) {
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
  }

  function niceStep(target) {
    const decade = 10 ** Math.floor(Math.log10(target));
    const units = target / decade;
    return decade * (units < 1.5 ? 1 : units < 3.5 ? 2 : units < 7.5 ? 5 : 10);
  }

  function drawGrid(ctx, project, run) {
    const [cx, cy] = run.center;
    const step = niceStep(run.extent / 7);
    const floor = run.min[2] - step * 0.4;
    const halfWidth = Math.max(3, Math.ceil(run.extent * 0.62 / step));
    const x0 = Math.round(cx / step) * step, y0 = Math.round(cy / step) * step;
    const edge = halfWidth * step;
    ctx.beginPath();
    for (let i = -halfWidth; i <= halfWidth; ++i) {
      const x = x0 + step * i, y = y0 + step * i;
      line(ctx, project([x, y0 - edge, floor]), project([x, y0 + edge, floor]));
      line(ctx, project([x0 - edge, y, floor]), project([x0 + edge, y, floor]));
    }
    ctx.strokeStyle = palette.grid;
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  function drawAxes(ctx, height) {
    const origin = [50, height - 71];
    const project = makeProjection(view, origin, [0, 0, 0], 24);
    ctx.font = "12px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (const [i, axis] of axes.entries()) {
      ctx.beginPath();
      line(ctx, origin, project(axis));
      ctx.strokeStyle = palette.muted;
      ctx.lineWidth = 1.3;
      ctx.stroke();
      const [x, y] = project(axis.map(n => n * 1.4));
      ctx.fillStyle = palette.text;
      ctx.fillText("XYZ"[i], x, y);
    }
  }

  function drawScale(ctx, scale, width, height) {
    const value = niceStep(70 / scale);
    const start = width - 17 - value * scale;
    const y = height - 21;
    ctx.beginPath();
    line(ctx, [start, y], [width - 17, y]);
    for (const x of [start, width - 17]) line(ctx, [x, y - 4], [x, y + 3]);
    ctx.strokeStyle = palette.muted;
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    ctx.fillStyle = palette.muted;
    ctx.font = "12px system-ui, sans-serif";
    ctx.fillText(value.toLocaleString("zh-CN", { maximumSignificantDigits: 3 }) + " m", (start + width - 17) / 2, y - 6);
  }

  function drawPath(ctx, frames, project, until, fill, lineWidth) {
    ctx.beginPath();
    for (let i = 0; i < until; ++i) {
      const [x, y] = project(frames[i].p);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = fill;
    ctx.lineWidth = lineWidth;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.stroke();
    if (until === 1) {
      const [x, y] = project(frames[0].p);
      ctx.beginPath();
      ctx.arc(x, y, 2, 0, Math.PI * 2);
      ctx.fillStyle = fill;
      ctx.fill();
    }
  }

  // Rotate the sensor's local axes by its stored camera/body-to-world quaternion.
  function worldPoint(frame, [x, y, z]) {
    const [a, b, c, w] = frame.q;
    const u = 2 * (b * z - c * y);
    const v = 2 * (c * x - a * z);
    const s = 2 * (a * y - b * x);
    return [
      frame.p[0] + x + w * u + b * s - c * v,
      frame.p[1] + y + w * v + c * u - a * s,
      frame.p[2] + z + w * s + a * v - b * u,
    ];
  }

  function drawPose(ctx, frame, size, project) {
    const origin = project(frame.p);
    const corners = [[-0.7, -0.5, 1], [0.7, -0.5, 1], [0.7, 0.5, 1], [-0.7, 0.5, 1]]
      .map(vertex => project(worldPoint(frame, vertex.map(n => n * size))));
    ctx.beginPath();
    for (let i = 0; i < corners.length; ++i) {
      line(ctx, origin, corners[i]);
      line(ctx, corners[i], corners[(i + 1) % corners.length]);
    }
    ctx.strokeStyle = palette.text;
    ctx.lineWidth = 1.45;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(origin[0], origin[1], 4.5, 0, Math.PI * 2);
    ctx.fillStyle = palette.path;
    ctx.fill();
  }

  function drawScene(run, frame, frameOffset) {
    const { ctx, width, height } = prepareCanvas(ui.scene);
    if (!run) {
      ctx.fillStyle = palette.muted;
      ctx.font = "15px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("此文件没有有效位姿", width / 2, height / 2);
      return;
    }

    const scale = Math.min(width, height) * 0.63 / run.extent * view.zoom;
    const project = makeProjection(view, [width / 2, height * 0.50], run.center, scale);
    drawGrid(ctx, project, run);
    drawPath(ctx, run.frames, project, run.frames.length, palette.preview, 1.45);
    if (frame) {
      drawPath(ctx, run.frames, project, frameOffset - run.first + 1, palette.path, 2.15);
      drawPose(ctx, frame, run.extent * 0.055, project);
    }
    drawAxes(ctx, height);
    drawScale(ctx, scale, width, height);
  }

  function labelSeconds(value) {
    return value.toFixed(2) + " s";
  }

  function render() {
    if (!track) return;
    const offset = containingFrame(time);
    const frame = track.frames[offset];
    const missingTimestamp = time > frameEnd(offset) + 1e-9;
    const valid = Boolean(frame.p) && !missingTimestamp;
    const run = valid ? track.segments.get(frame.segment) : fallbackSegment(offset);
    const prefix = valid ? "有效区段" : "区段预览";
    ui.segment.textContent = run
      ? prefix + " " + labelSeconds(run.frames[0].t) + " – " + labelSeconds(run.frames.at(-1).t)
      : "有效区段 · 无";
    ui.clock.textContent = labelSeconds(time) + " / " + labelSeconds(track.duration);
    ui.position.value = String(time);
    ui.position.setAttribute("aria-valuetext", labelSeconds(time));
    ui.validity.textContent = valid ? "有效" : (missingTimestamp ? "时间戳空段" : "无有效位姿");
    ui.validity.dataset.valid = String(valid);
    ui.frame.textContent = missingTimestamp ? "—" : String(frame.index);
    ui.coordinates.textContent = valid
      ? frame.p.map((value, i) => "XYZ"[i] + " " + value.toFixed(3)).join("  ·  ")
      : "—";
    drawScene(run, valid ? frame : null, offset);
  }

  function pause() {
    if (animation !== null) cancelAnimationFrame(animation);
    animation = null;
    ui.play.textContent = "播放";
  }

  function rebase() {
    startTime = time;
    startWall = performance.now();
  }

  function tick(now) {
    time = clamp(startTime + (now - startWall) / 1000 * speed, 0, track.duration);
    render();
    if (time === track.duration) pause();
    else animation = requestAnimationFrame(tick);
  }

  function resetView() {
    Object.assign(view, viewDefaults);
    render();
  }

  function rotate(dx, dy) {
    view.yaw += dx * 0.009;
    view.pitch = clamp(view.pitch + dy * 0.009, -Math.PI * 0.46, Math.PI * 0.46);
    render();
  }

  ui.play.addEventListener("click", () => {
    if (animation !== null) {
      pause();
    } else {
      if (time >= track.duration) time = 0;
      rebase();
      animation = requestAnimationFrame(tick);
      ui.play.textContent = "暂停";
    }
  });

  ui.restart.addEventListener("click", () => {
    pause();
    time = 0;
    render();
  });

  ui.position.addEventListener("input", () => {
    time = clamp(Number(ui.position.value), 0, track.duration);
    rebase();
    render();
  });

  ui.speed.addEventListener("change", () => {
    speed = Number(ui.speed.value);
    rebase();
  });

  ui.tracks.addEventListener("change", () => selectTrack(ui.tracks.value));
  ui.resetView.addEventListener("click", resetView);

  ui.scene.addEventListener("pointerdown", event => {
    if (event.button !== 0 || pointer) return;
    pointer = { id: event.pointerId, x: event.clientX, y: event.clientY };
    ui.scene.setPointerCapture(event.pointerId);
    ui.scene.classList.add("dragging");
  });

  ui.scene.addEventListener("pointermove", event => {
    if (!pointer || pointer.id !== event.pointerId) return;
    rotate(event.clientX - pointer.x, event.clientY - pointer.y);
    pointer.x = event.clientX;
    pointer.y = event.clientY;
  });

  ui.scene.addEventListener("lostpointercapture", () => {
    pointer = null;
    ui.scene.classList.remove("dragging");
  });

  ui.scene.addEventListener("wheel", event => {
    event.preventDefault();
    const units = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? 240 : 1;
    view.zoom = clamp(view.zoom * Math.exp(-event.deltaY * units * 0.0015), 0.4, 5);
    render();
  }, { passive: false });

  ui.scene.addEventListener("keydown", event => {
    const deltas = { ArrowLeft: [-15, 0], ArrowRight: [15, 0], ArrowUp: [0, -12], ArrowDown: [0, 12] };
    if (!Object.hasOwn(deltas, event.key)) return;
    event.preventDefault();
    rotate(...deltas[event.key]);
  });

  let importCount = 0;
  ui.file.addEventListener("change", async () => {
    const file = ui.file.files[0];
    if (!file) return;
    const format = ui.format.value;
    ui.file.disabled = true;
    showStatus("正在读取位姿…");
    try {
      const parsed = VioPoseIO.parse(await file.text(), format, file.name);
      const entry = buildTrack(parsed);
      const id = "import-" + (++importCount);
      ui.tracks.add(new Option(parsed.name, id));
      tracks.set(id, entry);
      ui.tracks.value = id;
      selectTrack(id);
      const count = entry.runs.reduce((sum, segment) => sum + segment.frames.length, 0);
      showStatus(parsed.name + " · " + count.toLocaleString("zh-CN") + " / " +
        entry.frames.length.toLocaleString("zh-CN") + " 条有效位姿");
    } catch (error) {
      showStatus(error.message, true);
    } finally {
      ui.file.value = "";
      ui.file.disabled = false;
    }
  });

  try {
    for (const [id, csv] of Object.entries(VioReplayExamples)) {
      tracks.set(id, buildTrack(VioPoseIO.parse(csv, "csv", id)));
    }
    selectTrack(ui.tracks.value);
    new ResizeObserver(() => {
      render();
      drawCoverage();
    }).observe(ui.scene);
  } catch (error) {
    showStatus("内置轨迹加载失败 · " + error.message, true);
    ui.play.disabled = ui.restart.disabled = ui.position.disabled = true;
  }
})();
