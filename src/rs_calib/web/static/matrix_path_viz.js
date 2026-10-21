/**
 * Matrix-path 3D canvas (Cell A · Elite EC616), adapted from sensors-arm viz.
 * Robot base frame (Z-up) ↔ Three.js (Y-up) via robotToThree / RPY mapping.
 */
(function (global) {
  'use strict';

  const EC616 = {
    urdf: 'models/ec616/ec616.urdf',
    joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
    joint_offset_rad: [0.0, -1.57079632679, 0.0, -1.57079632679, 3.14159265359, 0.0],
    joint_signs: [1, 1, 1, 1, 1, 1],
    mirror_y: true,
    home_joints_rad: [0, -0.4, 0.8, 0, 1.2, 0],
  };

  function robotToThree(x, y, z) {
    return { x: x, y: z, z: -y };
  }

  function softMaterials(root) {
    root.traverse(function (c) {
      if (!c.isMesh || !c.material) return;
      const mats = Array.isArray(c.material) ? c.material : [c.material];
      mats.forEach(function (m) {
        if (m && m.side != null) m.side = THREE.DoubleSide;
      });
    });
  }

  function poseToThree(xyzrpy) {
    const p = robotToThree(Number(xyzrpy[0]), Number(xyzrpy[1]), Number(xyzrpy[2]));
    const rx = Number(xyzrpy[3]) || 0;
    const ry = Number(xyzrpy[4]) || 0;
    const rz = Number(xyzrpy[5]) || 0;
    return { pos: p, euler: { x: rx, y: rz, z: -ry } };
  }

  function MatrixPathViz(host, opts) {
    opts = opts || {};
    this.host = host;
    this.assetBase = (opts.assetBase || '').replace(/\/?$/, '/');
    this._disposed = false;
    this._waypoints = [];
    this._activeIndex = -1;
    this._robot = null;
    this._loadToken = 0;
    this._pathGroup = null;
    this._beadGeom = null;
    this._seedMarker = null;
    this._flangeMarker = null;
    this._hintEl = opts.hintEl || null;
    this._initThree();
    this._loadUrdf();
  }

  MatrixPathViz.prototype._setHint = function (msg) {
    if (this._hintEl) this._hintEl.textContent = msg;
  };

  MatrixPathViz.prototype._asset = function (rel) {
    return this.assetBase + String(rel).replace(/^\//, '');
  };

  MatrixPathViz.prototype._initThree = function () {
    const host = this.host;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070c14);
    const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
    camera.position.set(1.15, 0.85, 1.15);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    host.innerHTML = '';
    host.appendChild(renderer.domElement);
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    renderer.domElement.style.display = 'block';

    let controls = null;
    if (typeof THREE.OrbitControls === 'function') {
      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.target.set(0, 0.35, 0);
      controls.enableDamping = true;
      controls.dampingFactor = 0.08;
      controls.minDistance = 0.2;
      controls.maxDistance = 8;
      controls.update();
    }

    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const dir = new THREE.DirectionalLight(0xffffff, 0.85);
    dir.position.set(2, 3, 1);
    scene.add(dir);
    const grid = new THREE.GridHelper(2, 20, 0x3dd6c6, 0x1a2740);
    scene.add(grid);

    const axes = new THREE.AxesHelper(0.25);
    scene.add(axes);

    this.scene = scene;
    this.camera = camera;
    this.renderer = renderer;
    this.controls = controls;
    this._beadGeom = new THREE.SphereGeometry(0.006, 12, 12);
    this._pathGroup = new THREE.Group();
    scene.add(this._pathGroup);

    this._seedMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.014, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xf0b429 })
    );
    this._seedMarker.add(new THREE.AxesHelper(0.05));
    this._seedMarker.visible = false;
    scene.add(this._seedMarker);

    this._flangeMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 14, 14),
      new THREE.MeshBasicMaterial({ color: 0x3b82f6 })
    );
    this._flangeMarker.add(new THREE.AxesHelper(0.04));
    this._flangeMarker.visible = false;
    scene.add(this._flangeMarker);

    const self = this;
    this._paused = false;
    this._resize = function resize() {
      const w = Math.max(1, host.clientWidth);
      const h = Math.max(1, host.clientHeight);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h, false);
    };
    this._resize();
    window.addEventListener('resize', this._resize);
    if (typeof ResizeObserver === 'function') {
      this._ro = new ResizeObserver(this._resize);
      this._ro.observe(host);
    }
    (function loop() {
      if (self._disposed) return;
      requestAnimationFrame(loop);
      if (self._paused) return;
      if (self.controls && self.controls.enableDamping) self.controls.update();
      self.renderer.render(self.scene, self.camera);
    })();
  };

  MatrixPathViz.prototype.setPaused = function (paused) {
    this._paused = !!paused;
    if (!this._paused && typeof this._resize === 'function') {
      this._resize();
    }
  };

  MatrixPathViz.prototype.resize = function () {
    if (typeof this._resize === 'function') this._resize();
  };

  MatrixPathViz.prototype._loadUrdf = function () {
    if (typeof URDFLoader === 'undefined') {
      this._setHint('URDFLoader 未加载 — 仅显示点位');
      return;
    }
    const token = ++this._loadToken;
    const urdfUrl = this._asset(EC616.urdf);
    this._setHint('加载 Cell A · EC616 …');
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.workingPath = urdfUrl.replace(/[^/]+$/, '');
    const self = this;
    loader.load(
      urdfUrl,
      function (robot) {
        if (token !== self._loadToken || self._disposed) return;
        softMaterials(robot);
        robot.rotation.x = -Math.PI / 2;
        if (EC616.mirror_y) robot.scale.y = -1;
        if (self._robot) self.scene.remove(self._robot);
        self._robot = robot;
        self.scene.add(robot);
        self.setJoints(EC616.home_joints_rad);
        self._setHint('Cell A · Elite EC616 · 矩阵点位预览（琥珀=种子 蓝=当前 青=路径）');
      },
      undefined,
      function (err) {
        if (token !== self._loadToken) return;
        self._setHint('URDF 失败: ' + (err && err.message ? err.message : err));
      }
    );
  };

  MatrixPathViz.prototype.setJoints = function (jointsRad) {
    const robot = this._robot;
    if (!robot || !robot.joints || !Array.isArray(jointsRad)) return 0;
    const names = EC616.joint_names;
    const offs = EC616.joint_offset_rad;
    const signs = EC616.joint_signs;
    let n = 0;
    for (let i = 0; i < Math.min(6, jointsRad.length, names.length); i++) {
      const joint = robot.joints[names[i]];
      if (!joint || typeof joint.setJointValue !== 'function') continue;
      if (joint.limit && Number(joint.limit.lower) === 0 && Number(joint.limit.upper) === 0) {
        joint.limit.lower = -Math.PI * 2;
        joint.limit.upper = Math.PI * 2;
      }
      const q = (signs[i] == null ? 1 : Number(signs[i])) * (Number(jointsRad[i]) - Number(offs[i] || 0));
      joint.setJointValue(q);
      n += 1;
    }
    robot.updateMatrixWorld(true);
    return n;
  };

  MatrixPathViz.prototype._clearPath = function () {
    if (!this._pathGroup) return;
    while (this._pathGroup.children.length) {
      const ch = this._pathGroup.children[0];
      this._pathGroup.remove(ch);
      if (ch.geometry && ch !== this._beadGeom && typeof ch.geometry.dispose === 'function') {
        try {
          ch.geometry.dispose();
        } catch (_) {}
      }
      if (ch.material && typeof ch.material.dispose === 'function') {
        try {
          ch.material.dispose();
        } catch (_) {}
      }
    }
  };

  MatrixPathViz.prototype.setWaypoints = function (waypoints, seedXyzrpy) {
    this._waypoints = Array.isArray(waypoints) ? waypoints.slice() : [];
    this._activeIndex = -1;
    this._clearPath();

    if (seedXyzrpy && seedXyzrpy.length >= 6) {
      const s = poseToThree(seedXyzrpy);
      this._seedMarker.position.set(s.pos.x, s.pos.y, s.pos.z);
      this._seedMarker.rotation.set(s.euler.x, s.euler.y, s.euler.z, 'XYZ');
      this._seedMarker.visible = true;
    }

    const n = this._waypoints.length;
    if (!n) {
      this._setHint('尚无点位 — 生成矩阵后显示');
      return;
    }

    const positions = [];
    for (let i = 0; i < n; i++) {
      const wp = this._waypoints[i];
      const xyzrpy = wp.xyzrpy || wp;
      const t = poseToThree(xyzrpy);
      positions.push(t.pos.x, t.pos.y, t.pos.z);

      const bead = new THREE.Mesh(
        this._beadGeom,
        new THREE.MeshBasicMaterial({ color: 0x3dd6c6 })
      );
      bead.position.set(t.pos.x, t.pos.y, t.pos.z);
      bead.userData.index = i;
      bead.raycast = function () {};
      this._pathGroup.add(bead);

      // Orientation triad every few points (or all if few)
      if (n <= 36 || i % Math.ceil(n / 24) === 0 || i === n - 1) {
        const frame = new THREE.AxesHelper(0.028);
        frame.position.set(t.pos.x, t.pos.y, t.pos.z);
        frame.rotation.set(t.euler.x, t.euler.y, t.euler.z, 'XYZ');
        frame.raycast = function () {};
        this._pathGroup.add(frame);
      }
    }

    const lineGeom = new THREE.BufferGeometry();
    lineGeom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    const line = new THREE.Line(
      lineGeom,
      new THREE.LineBasicMaterial({ color: 0x3dd6c6, transparent: true, opacity: 0.75 })
    );
    line.raycast = function () {};
    this._pathGroup.add(line);

    this._fitToPath(positions);
    this._setHint('矩阵点位 ' + n + ' · 琥珀种子 · 拖拽旋转视角');
  };

  MatrixPathViz.prototype._fitToPath = function (flatPositions) {
    if (!flatPositions || flatPositions.length < 3 || !this.controls) return;
    let minX = Infinity,
      minY = Infinity,
      minZ = Infinity;
    let maxX = -Infinity,
      maxY = -Infinity,
      maxZ = -Infinity;
    for (let i = 0; i < flatPositions.length; i += 3) {
      minX = Math.min(minX, flatPositions[i]);
      minY = Math.min(minY, flatPositions[i + 1]);
      minZ = Math.min(minZ, flatPositions[i + 2]);
      maxX = Math.max(maxX, flatPositions[i]);
      maxY = Math.max(maxY, flatPositions[i + 1]);
      maxZ = Math.max(maxZ, flatPositions[i + 2]);
    }
    const cx = 0.5 * (minX + maxX);
    const cy = 0.5 * (minY + maxY);
    const cz = 0.5 * (minZ + maxZ);
    const span = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 0.15);
    this.controls.target.set(cx, cy, cz);
    this.camera.position.set(cx + span * 1.6, cy + span * 1.1, cz + span * 1.6);
    this.controls.update();
  };

  MatrixPathViz.prototype.setProgress = function (index) {
    this._activeIndex = Number(index);
    if (!this._pathGroup) return;
    const active = this._activeIndex;
    this._pathGroup.children.forEach(function (ch) {
      if (!ch.isMesh || ch.userData.index == null) return;
      const i = ch.userData.index;
      const mat = ch.material;
      if (!mat || !mat.color) return;
      if (i === active) mat.color.setHex(0xf0b429);
      else if (i < active) mat.color.setHex(0x7fd99a);
      else mat.color.setHex(0x3dd6c6);
    });
    if (active >= 0 && active < this._waypoints.length) {
      const wp = this._waypoints[active];
      const xyzrpy = wp.xyzrpy || wp;
      const t = poseToThree(xyzrpy);
      this._flangeMarker.position.set(t.pos.x, t.pos.y, t.pos.z);
      this._flangeMarker.rotation.set(t.euler.x, t.euler.y, t.euler.z, 'XYZ');
      this._flangeMarker.visible = true;
    }
  };

  MatrixPathViz.prototype.setLivePose = function (xyzrpy) {
    if (!xyzrpy || xyzrpy.length < 6) {
      this._flangeMarker.visible = false;
      return;
    }
    const t = poseToThree(xyzrpy);
    this._flangeMarker.position.set(t.pos.x, t.pos.y, t.pos.z);
    this._flangeMarker.rotation.set(t.euler.x, t.euler.y, t.euler.z, 'XYZ');
    this._flangeMarker.visible = true;
  };

  MatrixPathViz.prototype.dispose = function () {
    this._disposed = true;
    this._clearPath();
    if (this._ro) {
      try {
        this._ro.disconnect();
      } catch (_) {}
    }
    if (this.renderer) {
      try {
        this.renderer.dispose();
      } catch (_) {}
      if (this.renderer.domElement && this.renderer.domElement.parentNode) {
        this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
      }
    }
  };

  global.MatrixPathViz = MatrixPathViz;
  global.MatrixPathVizEC616 = EC616;
})(typeof window !== 'undefined' ? window : this);
