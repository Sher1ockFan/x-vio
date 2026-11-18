(() => {
  "use strict";

  const vertexSource = `
    attribute vec3 a_position;
    attribute vec3 a_normal;
    attribute vec3 a_color;
    uniform mat3 u_localToWorld;
    uniform mat3 u_worldToView;
    uniform vec3 u_offset;
    uniform vec3 u_scale;
    varying vec3 v_normal;
    varying vec3 v_color;
    void main() {
      vec3 world = u_localToWorld * a_position + u_offset;
      vec3 view = u_worldToView * world;
      gl_Position = vec4(view * u_scale, 1.0);
      v_normal = u_worldToView * u_localToWorld * a_normal;
      v_color = a_color;
    }`;

  const fragmentSource = `
    precision mediump float;
    varying vec3 v_normal;
    varying vec3 v_color;
    void main() {
      vec3 normal = normalize(v_normal);
      vec3 direction = normalize(vec3(-0.48, 0.65, 0.62));
      float light = 0.57 + 0.45 * abs(dot(normal, direction));
      gl_FragColor = vec4(v_color * light, 1.0);
    }`;

  function unpack(base64) {
    return Uint8Array.from(atob(base64), byte => byte.charCodeAt(0));
  }

  function facetNormals(vertices) {
    const normals = new Float32Array(vertices.length);
    for (let i = 0; i < vertices.length; i += 9) {
      const ax = vertices[i+3]-vertices[i], ay = vertices[i+4]-vertices[i+1], az = vertices[i+5]-vertices[i+2];
      const bx = vertices[i+6]-vertices[i], by = vertices[i+7]-vertices[i+1], bz = vertices[i+8]-vertices[i+2];
      const x = ay*bz-az*by, y = az*bx-ax*bz, z = ax*by-ay*bx;
      const length = Math.hypot(x, y, z);
      if (!(length > 0)) throw new Error("夹爪网格含无效三角形。");
      for (let j = 0; j < 9; j += 3) normals.set([x/length, y/length, z/length], i+j);
    }
    return normals;
  }

  function shader(gl, type, source) {
    const result = gl.createShader(type);
    gl.shaderSource(result, source);
    gl.compileShader(result);
    if (!gl.getShaderParameter(result, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(result));
    return result;
  }

  function create(canvas, mesh) {
    const gl = canvas.getContext("webgl", { alpha: true, antialias: true, depth: true });
    if (!gl) throw new Error("当前浏览器无法开启 WebGL。");
    const positions = new Float32Array(unpack(mesh.positions).buffer);
    const colors = unpack(mesh.colors);
    if (!positions.length || positions.length % 9 || positions.length !== colors.length) {
      throw new Error("夹爪网格数据不完整。");
    }
    const program = gl.createProgram();
    gl.attachShader(program, shader(gl, gl.VERTEX_SHADER, vertexSource));
    gl.attachShader(program, shader(gl, gl.FRAGMENT_SHADER, fragmentSource));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
    gl.useProgram(program);

    const attributes = [
      ["a_position", positions, gl.FLOAT, false],
      ["a_normal", facetNormals(positions), gl.FLOAT, false],
      ["a_color", colors, gl.UNSIGNED_BYTE, true],
    ];
    for (const [name, values, type, normalized] of attributes) {
      const location = gl.getAttribLocation(program, name);
      gl.bindBuffer(gl.ARRAY_BUFFER, gl.createBuffer());
      gl.bufferData(gl.ARRAY_BUFFER, values, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(location);
      gl.vertexAttribPointer(location, 3, type, normalized, 0, 0);
    }
    const location = name => gl.getUniformLocation(program, name);
    const modelMatrix = location("u_localToWorld"), viewMatrix = location("u_worldToView");
    const translation = location("u_offset"), scaling = location("u_scale");
    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(0, 0, 0, 0);

    function draw(frame, view, center, extent, scale, width, height) {
      const ratio = Math.min(window.devicePixelRatio || 1, 2);
      const w = Math.max(1, Math.round(width * ratio)), h = Math.max(1, Math.round(height * ratio));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
      gl.viewport(0, 0, w, h);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      const c = Math.cos(view.yaw), s = Math.sin(view.yaw);
      const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
      // Use the same XY projection as the trajectory canvas, with Y up in NDC.
      gl.uniformMatrix3fv(viewMatrix, false, [
        c, -s*sp, s*cp,
        -s, -c*sp, c*cp,
        0, cp, sp,
      ]);
      const rotation = { p: [0, 0, 0], q: frame.q };
      const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
      gl.uniformMatrix3fv(modelMatrix, false, axes.flatMap(axis => VioTcp.worldPoint(rotation, axis)));
      gl.uniform3fv(translation, frame.p.map((value, axis) => value - center[axis]));
      gl.uniform3f(scaling, 2 * scale / width, 2 * scale / height, -1 / (extent * 3 + 1));
      gl.drawArrays(gl.TRIANGLES, 0, positions.length / 3);
    }

    return Object.freeze({ draw, triangles: positions.length / 9 });
  }

  globalThis.VioGripperRenderer = Object.freeze({ create });
})();
