(() => {
  "use strict";

  const cad = Object.freeze({
    tcpMm: Object.freeze([0, 82.48179, 35.535025]),
    lensApexMm: Object.freeze([0.05257675, -59.32598, 59.40503]),
    cameraAxesInCad: Object.freeze([
      Object.freeze([1, 0, 0]),
      Object.freeze([0, -0.16902267, -0.98561216]),
      Object.freeze([0, 0.98561216, -0.16902267]),
    ]),
  });
  const difference = cad.tcpMm.map((n, axis) => n - cad.lensApexMm[axis]);
  const nominalMm = Object.freeze(cad.cameraAxesInCad.map(axis =>
    axis.reduce((sum, n, i) => sum + n * difference[i], 0)));

  function offsetMetres(mm) {
    if (!Array.isArray(mm) || mm.length !== 3 || !mm.every(Number.isFinite)) {
      throw new Error("TCP 偏移需要三个有限的毫米数值。");
    }
    return mm.map(value => value / 1000);
  }

  // The quaternion maps the local camera or body axes into world axes.
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

  function tcpFrames(frames, offsetMm) {
    const offset = offsetMetres(offsetMm);
    return frames.map(frame =>
      frame.p ? { ...frame, p: worldPoint(frame, offset) } : frame);
  }

  globalThis.VioTcp = Object.freeze({ nominalMm, worldPoint, tcpFrames });
})();
