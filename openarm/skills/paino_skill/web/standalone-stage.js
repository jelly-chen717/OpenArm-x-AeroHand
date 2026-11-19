// Local Three.js stage. Layout, animation and shaders are authored for this player.
// Piano pitch/time data comes from the existing TXT playback engine.
function createPianoStage(host, readPlayback, audition) {
  const { firstWhite, whiteCount, minMidi, maxMidi } = keyboardRange;
  const halfKeyboard = whiteCount / 2;
  const panLimit = Math.max(0, halfKeyboard - 8.5);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#060607');
  const camera = new THREE.PerspectiveCamera(46, 1, 0.1, 220);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 1.5));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.setAttribute('aria-label', '立体钢琴与下落音符舞台，选择歌曲后可试听白键');
  renderer.domElement.setAttribute('role', 'img');
  host.append(renderer.domElement);

  scene.add(new THREE.HemisphereLight(0xfff4df, 0x383747, 0.6));
  const keyLight = new THREE.DirectionalLight(0xfff5de, 2.5);
  keyLight.position.set(-18, 16, 8);
  keyLight.castShadow = true;
  Object.assign(keyLight.shadow.camera, { left: -halfKeyboard-3, right: halfKeyboard+3, top: 14, bottom: -10, near: 1, far: 70 });
  keyLight.shadow.mapSize.set(2048, 1024);
  keyLight.shadow.bias = -0.0005;
  keyLight.shadow.normalBias = 0.012;
  scene.add(keyLight);
  const rimLight = new THREE.DirectionalLight(0xc5dcff, 1.3);
  rimLight.position.set(18, 8, -12);
  scene.add(rimLight);

  const keys = [];
  const whiteKeys = new Map();
  const hitColor = new THREE.Color('#f6c36b');
  const rightColor = new THREE.Color('#f29a58');
  const ivory = new THREE.Color('#e7e2d5');
  const black = new THREE.Color('#101113');
  const dummy = new THREE.Object3D();
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const keyLength = 8.6;

  function roundedShape(width, height, radius) {
    const shape = new THREE.Shape();
    const x = -width / 2, y = -height / 2, r = radius;
    shape.moveTo(x + r, y);
    shape.lineTo(x + width - r, y);
    shape.quadraticCurveTo(x + width, y, x + width, y + r);
    shape.lineTo(x + width, y + height - r);
    shape.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
    shape.lineTo(x + r, y + height);
    shape.quadraticCurveTo(x, y + height, x, y + height - r);
    shape.lineTo(x, y + r);
    shape.quadraticCurveTo(x, y, x + r, y);
    return shape;
  }

  const whiteGeometry = new THREE.ExtrudeGeometry(roundedShape(0.952, keyLength, 0.055), {
    depth: 0.16, bevelEnabled: true, bevelSegments: 2,
    steps: 1, bevelSize: 0.015, bevelThickness: 0.015, curveSegments: 3,
  });
  whiteGeometry.rotateX(Math.PI / 2);
  whiteGeometry.translate(0, 0, keyLength / 2);
  const bodyGeometry = new THREE.BoxGeometry(0.928, 0.38, keyLength - 0.05);
  bodyGeometry.translate(0, -0.35, keyLength / 2);
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: '#59432f', roughness: 0.48 });
  const blackLength = 5.35;
  // Small bevels catch the side light and reveal the raised key silhouette.
  const blackGeometry = new THREE.ExtrudeGeometry(roundedShape(0.49, blackLength - .08, .045), {
    depth: .46, bevelEnabled: true, bevelSegments: 3,
    steps: 1, bevelSize: .04, bevelThickness: .04, curveSegments: 4,
  });
  blackGeometry.rotateX(Math.PI / 2);
  blackGeometry.translate(0, .52, blackLength / 2);

  let whiteIndex = 0;
  for (let midi = minMidi; midi <= maxMidi; midi++) {
    const isBlack = [1, 3, 6, 8, 10].includes(midi % 12);
    const x = isBlack ? whiteIndex - halfKeyboard : whiteIndex++ - (whiteCount - 1) / 2;
    const material = new THREE.MeshStandardMaterial({
      color: isBlack ? black : ivory, roughness: isBlack ? 0.22 : 0.3,
      metalness: isBlack ? 0.12 : 0.02, emissive: '#000000',
    });
    const pivot = new THREE.Group();
    pivot.position.x = x;
    const mesh = new THREE.Mesh(isBlack ? blackGeometry : whiteGeometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    pivot.add(mesh);
    if (!isBlack) pivot.add(new THREE.Mesh(bodyGeometry, bodyMaterial));
    const number = isBlack ? null : firstWhite + whiteIndex - 1;
    const key = { midi, x, isBlack, number, pivot, mesh, material, press: 0, glow: 0, manualUntil: 0 };
    mesh.userData.key = key;
    keys.push(key);
    if (!isBlack) whiteKeys.set(number, key);
    scene.add(pivot);
  }

  const frameMaterial = new THREE.MeshStandardMaterial({ color: '#141315', roughness: 0.24, metalness: 0.48 });
  const bed = new THREE.Mesh(new THREE.BoxGeometry(whiteCount + 1.4, 0.5, keyLength + .75), frameMaterial);
  bed.position.set(0, -0.79, keyLength / 2);
  bed.receiveShadow = true;
  scene.add(bed);
  for (const x of [-halfKeyboard - .4, halfKeyboard + .4]) {
    const cheek = new THREE.Mesh(new THREE.BoxGeometry(0.72, 0.62, keyLength + .5), frameMaterial);
    cheek.position.set(x, -0.1, keyLength / 2);
    cheek.castShadow = true;
    scene.add(cheek);
  }
  const rail = new THREE.Mesh(new THREE.BoxGeometry(whiteCount + 1.4, 0.48, 0.35), frameMaterial);
  rail.position.set(0, -0.08, -0.23);
  scene.add(rail);

  // The same x positions drive notes, key illumination and contact particles.
  const noteGeometry = new THREE.ExtrudeGeometry(roundedShape(0.82, 1, 0.08), {
    depth: 0.14, bevelEnabled: true, bevelSegments: 2, bevelSize: 0.025,
    bevelThickness: 0.025, steps: 1, curveSegments: 3,
  });
  const noteMaterial = new THREE.MeshStandardMaterial({
    color: '#8e6c31', roughness: 0.25, metalness: 0.35,
    emissive: '#f7bd5b', emissiveIntensity: 0.65,
  });
  noteMaterial.onBeforeCompile = shader => {
    shader.vertexShader = 'varying vec2 vNoteLocal;\n' + shader.vertexShader;
    shader.vertexShader = shader.vertexShader.replace('#include <begin_vertex>',
      '#include <begin_vertex>\nvNoteLocal = position.xy;');
    shader.fragmentShader = 'varying vec2 vNoteLocal;\n' + shader.fragmentShader;
    shader.fragmentShader = shader.fragmentShader.replace('#include <emissivemap_fragment>', `
      #include <emissivemap_fragment>
      float edge = max(smoothstep(0.34, 0.41, abs(vNoteLocal.x)), smoothstep(0.42, 0.5, abs(vNoteLocal.y)));
      totalEmissiveRadiance += vec3(1.0, 0.67, 0.26) * edge * 1.2;
    `);
  };
  const notes = new THREE.InstancedMesh(noteGeometry, noteMaterial, 1024);
  notes.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  notes.frustumCulled = false;
  notes.count = 0;
  scene.add(notes);

  const lines = [];
  for (let i = 0; i <= whiteCount; i++) lines.push(i - halfKeyboard, 0, -0.3, i - halfKeyboard, 35, -0.3);
  const trackGeometry = new THREE.BufferGeometry();
  trackGeometry.setAttribute('position', new THREE.Float32BufferAttribute(lines, 3));
  scene.add(new THREE.LineSegments(trackGeometry, new THREE.LineBasicMaterial({ color: '#62543c', transparent: true, opacity: 0.065 })));
  const line = new THREE.Mesh(new THREE.BoxGeometry(whiteCount, 0.022, 0.035), new THREE.MeshBasicMaterial({ color: new THREE.Color(1.2, 0.65, 0.19), toneMapped: false }));
  line.position.set(0, 0.045, 0.05);
  scene.add(line);

  // A soft additive contact plume. It is geometry in the scene, so it stays
  // aligned with the keyboard when the viewport's aspect ratio changes.
  const contactMaterial = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
    uniforms: { tint: { value: hitColor }, strength: { value: 0 } },
    vertexShader: 'varying vec2 vUv; void main(){vUv=uv;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);}',
    fragmentShader: `varying vec2 vUv; uniform vec3 tint; uniform float strength;
      void main(){float core=exp(-pow((vUv.x-.5)*5.0,2.0));
      float fade=pow(1.0-vUv.y,2.2);gl_FragColor=vec4(tint*2.0,core*fade*strength*.42);}`,
  });
  const contacts = [];
  for (const key of whiteKeys.values()) {
    const material = contactMaterial.clone();
    const plume = new THREE.Mesh(new THREE.PlaneGeometry(2.5, 4.8), material);
    plume.position.set(key.x, 2.4, 0.17);
    scene.add(plume);
    contacts.push({ key, plume, material });
  }
  const hitLights = [];
  for (let i = 0; i < 4; i++) {
    const light = new THREE.PointLight('#ffcd79', 0, 11, 2);
    light.position.set(0, 1.4, 0.8);
    scene.add(light);
    hitLights.push(light);
  }

  const particleLimit = 480;
  const particles = [];
  const particlePositions = new Float32Array(particleLimit * 3);
  const particleColors = new Float32Array(particleLimit * 3);
  const particleSizes = new Float32Array(particleLimit);
  const particleAlpha = new Float32Array(particleLimit);
  const particleGeometry = new THREE.BufferGeometry();
  particleGeometry.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));
  particleGeometry.setAttribute('color', new THREE.BufferAttribute(particleColors, 3));
  particleGeometry.setAttribute('size', new THREE.BufferAttribute(particleSizes, 1));
  particleGeometry.setAttribute('alpha', new THREE.BufferAttribute(particleAlpha, 1));
  const particleMaterial = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
    uniforms: { height: { value: 900 } },
    vertexShader: `attribute float size; attribute float alpha; attribute vec3 color;
      uniform float height; varying float vAlpha; varying vec3 vColor;
      void main(){vec4 mv=modelViewMatrix*vec4(position,1.0);vAlpha=alpha;vColor=color;
      gl_PointSize=clamp(size*height/-mv.z,1.0,90.0);gl_Position=projectionMatrix*mv;}`,
    fragmentShader: `varying float vAlpha; varying vec3 vColor;
      void main(){float r=length(gl_PointCoord-.5)*2.0;if(r>1.0)discard;
      gl_FragColor=vec4(vColor,exp(-r*r*5.0)*vAlpha);}`,
  });
  const particleMesh = new THREE.Points(particleGeometry, particleMaterial);
  particleMesh.frustumCulled = false;
  scene.add(particleMesh);

  const dustPositions = [];
  for (let i = 0; i < 170; i++) dustPositions.push((Math.random() - .5) * 62, Math.random() * 32, -Math.random() * 9 - 1);
  const dustGeometry = new THREE.BufferGeometry();
  dustGeometry.setAttribute('position', new THREE.Float32BufferAttribute(dustPositions, 3));
  scene.add(new THREE.Points(dustGeometry, new THREE.PointsMaterial({ color: '#ad8449', size: 0.035, transparent: true, opacity: 0.45 })));

  function emit(key, color, burst) {
    const amount = burst ? 12 : 1;
    for (let i = 0; i < amount && particles.length < particleLimit; i++) {
      const fog = i % 5 === 0;
      particles.push({ x: key.x + (Math.random() - .5) * .6, y: .12, z: .25,
        vx: (Math.random() - .5) * (fog ? .8 : 2.8), vy: 1.4 + Math.random() * 3,
        age: 0, life: fog ? 1.9 : .5 + Math.random() * .8,
        size: fog ? 1.1 : .045 + Math.random() * .09, fog, color });
    }
  }

  // Half-resolution bloom uses three small full-screen passes. The scene
  // remains sharp, while only bright fragments contribute to the glow.
  const sceneTarget = new THREE.WebGLRenderTarget(1, 1, { type: THREE.HalfFloatType });
  const bloomA = new THREE.WebGLRenderTarget(1, 1, { type: THREE.HalfFloatType, depthBuffer: false });
  const bloomB = new THREE.WebGLRenderTarget(1, 1, { type: THREE.HalfFloatType, depthBuffer: false });
  const postScene = new THREE.Scene();
  const postCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
  const postVertex = 'varying vec2 vUv; void main(){vUv=uv;gl_Position=vec4(position.xy,0.,1.);}';
  const blurMaterial = new THREE.ShaderMaterial({
    depthTest: false, depthWrite: false,
    uniforms: { image: { value: null }, step: { value: new THREE.Vector2() }, extract: { value: 0 } },
    vertexShader: postVertex,
    fragmentShader: `varying vec2 vUv; uniform sampler2D image; uniform vec2 step; uniform float extract;
      vec3 tap(vec2 uv){vec3 c=texture2D(image,uv).rgb;
      return mix(c,c*smoothstep(1.2,2.4,max(c.r,max(c.g,c.b))),extract);}
      void main(){vec3 c=tap(vUv)*.227027;c+=(tap(vUv+step*1.384615)+tap(vUv-step*1.384615))*.316216;
      c+=(tap(vUv+step*3.230769)+tap(vUv-step*3.230769))*.070270;gl_FragColor=vec4(c,1.);}`,
  });
  const outputMaterial = new THREE.ShaderMaterial({
    depthTest: false, depthWrite: false,
    uniforms: { image: { value: sceneTarget.texture }, bloom: { value: bloomA.texture } },
    vertexShader: postVertex,
    fragmentShader: `varying vec2 vUv; uniform sampler2D image; uniform sampler2D bloom;
      void main(){vec3 c=texture2D(image,vUv).rgb+texture2D(bloom,vUv).rgb*.5;
      c=clamp((c*(2.51*c+.03))/(c*(2.43*c+.59)+.14),0.,1.);
      c=pow(c,vec3(1./2.2));float vignette=1.-.15*pow(length((vUv-.5)*1.25),2.);
      gl_FragColor=vec4(c*vignette,1.);}`,
  });
  const quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), blurMaterial);
  postScene.add(quad);
  let width = 1, height = 1, fallSpeed = 7.2;
  let viewCenter = 0;
  let manualPan = false;
  let dragState = null;
  function resize() {
    width = Math.max(1, host.clientWidth);
    height = Math.max(1, host.clientHeight);
    renderer.setSize(width, height);
    camera.aspect = width / height;
    // Frame the front edge of the keyboard, which is closest to the camera.
    // Wide windows should enlarge the instrument instead of hitting a fixed
    // distance floor. Phones retain their movable close-up keyboard view.
    const portrait = camera.aspect < .85;
    // A longer desktop lens limits distortion across the full keyboard.
    camera.fov = portrait ? 46 : 26;
    const halfWidth = portrait ? 8.5 : halfKeyboard + 1.7;
    const elevation = THREE.MathUtils.degToRad(18);
    const horizontal = Math.cos(elevation);
    const vertical = Math.sin(elevation);
    const distance = portrait
      ? Math.max(30, halfWidth / (Math.tan(camera.fov * Math.PI / 360) * camera.aspect))
      : Math.max(22, (keyLength - 1) * horizontal + halfWidth / (Math.tan(camera.fov * Math.PI / 360) * camera.aspect));
    const halfVisible = distance * Math.tan(camera.fov * Math.PI / 360) / horizontal;
    const targetY = halfVisible * (portrait ? .32 : .24);
    camera.position.set(portrait ? viewCenter : 0, targetY + distance * vertical, 1 + distance * horizontal);
    camera.lookAt(portrait ? viewCenter : 0, targetY, 1);
    fallSpeed = halfVisible * 1.32 / 3.4;
    camera.updateProjectionMatrix();
    const ratio = renderer.getPixelRatio();
    sceneTarget.setSize(Math.round(width * ratio), Math.round(height * ratio));
    bloomA.setSize(Math.max(1, Math.round(width * ratio / 2)), Math.max(1, Math.round(height * ratio / 2)));
    bloomB.setSize(bloomA.width, bloomA.height);
    particleMaterial.uniforms.height.value = height * ratio;
  }
  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(host);
  resize();

  function composite() {
    renderer.setRenderTarget(sceneTarget);
    renderer.render(scene, camera);
    quad.material = blurMaterial;
    blurMaterial.uniforms.image.value = sceneTarget.texture;
    blurMaterial.uniforms.extract.value = 1;
    blurMaterial.uniforms.step.value.set(2.4 / bloomA.width, 0);
    renderer.setRenderTarget(bloomB);
    renderer.render(postScene, postCamera);
    blurMaterial.uniforms.image.value = bloomB.texture;
    blurMaterial.uniforms.extract.value = 0;
    blurMaterial.uniforms.step.value.set(0, 2.4 / bloomA.height);
    renderer.setRenderTarget(bloomA);
    renderer.render(postScene, postCamera);
    quad.material = outputMaterial;
    renderer.setRenderTarget(null);
    renderer.render(postScene, postCamera);
  }

  let previousTime = 0, previousScore = null, previousSongTime = 0;
  let activeBefore = new Set();
  let lastLabels = '';
  let frameCount = 0;
  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)');
  function render(nowMs) {
    if (document.hidden || nowMs - previousTime < 1000 / 45) return;
    const dt = Math.min(.05, (nowMs - previousTime) / 1000 || .016);
    previousTime = nowMs;
    const playback = readPlayback();
    const score = playback.score;
    const time = playback.time;
    if (score !== previousScore || time < previousSongTime || Math.abs(time - previousSongTime) > 600000) {
      particles.length = 0;
      activeBefore.clear();
      manualPan = false;
    }
    previousScore = score;
    previousSongTime = time;
    if (camera.aspect < .85 && score && !manualPan) {
      // Portrait shows a playable section at a useful scale. Follow the
      // current phrase in world space, keeping note/key alignment exact.
      const phrase = score.notes.filter(n => n.end >= time && n.start <= time + 1000000);
      if (phrase.length) {
        const xs = phrase.map(n => whiteKeys.get(n.key).x);
        const target = THREE.MathUtils.clamp((Math.min(...xs) + Math.max(...xs)) / 2, -panLimit, panLimit);
        viewCenter += (target - viewCenter) * (1 - Math.exp(-dt / .35));
      }
      // Translation preserves the viewing direction and the note/key alignment.
      camera.position.x = viewCenter;
    }
    const active = new Map();
    let noteCount = 0;
    if (score) {
      for (const note of score.notes) {
        if (note.start > time + 3400000) break;
        if (note.end < time) continue;
        const key = whiteKeys.get(note.key);
        const color = note.side === 'right' ? rightColor : hitColor;
        if (playback.sounding && note.start <= time && note.end > time) {
          active.set(note.key, { note, color });
          if (playback.playing && !activeBefore.has(note)) emit(key, color, true);
        }
        if (noteCount >= 1024) continue;
        const bottom = Math.max(.05, (note.start - time) / 1e6 * fallSpeed);
        const top = (note.end - time) / 1e6 * fallSpeed;
        const length = Math.max(.14, top - bottom);
        dummy.position.set(key.x, bottom + length / 2, -.04);
        dummy.scale.set(1, length, 1);
        dummy.rotation.set(0, 0, 0);
        dummy.updateMatrix();
        notes.setMatrixAt(noteCount, dummy.matrix);
        notes.setColorAt(noteCount++, color);
      }
    }
    notes.count = noteCount;
    notes.instanceMatrix.needsUpdate = true;
    if (notes.instanceColor) notes.instanceColor.needsUpdate = true;
    activeBefore = new Set([...active.values()].map(a => a.note));

    const lit = [];
    for (const key of keys) {
      const event = active.get(key.number);
      const pressed = !!event || key.manualUntil > nowMs;
      const color = event?.color || hitColor;
      const response = 1 - Math.exp(-dt / (pressed ? .025 : .09));
      key.press += ((pressed ? 1 : 0) - key.press) * response;
      key.glow += ((pressed ? 1 : 0) - key.glow) * (1 - Math.exp(-dt / (pressed ? .025 : .23)));
      key.pivot.rotation.x = key.press * (key.isBlack ? .022 : .025);
      key.pivot.position.y = -key.press * .025;
      key.material.color.copy(key.isBlack ? black : ivory).lerp(color, key.glow * .18);
      key.material.emissive.copy(color).multiplyScalar(key.glow * (key.isBlack ? 1.1 : .7));
      if (key.glow > .02) lit.push(key);
    }
    lit.sort((a, b) => b.glow - a.glow);
    for (let i = 0; i < hitLights.length; i++) {
      const key = lit[i];
      hitLights[i].intensity = key ? key.glow * 15 : 0;
      if (key) {
        hitLights[i].position.x = key.x;
        hitLights[i].color.copy(active.get(key.number)?.color || hitColor);
      }
    }
    for (const contact of contacts) {
      contact.material.uniforms.strength.value = contact.key.glow;
      contact.material.uniforms.tint.value.copy(active.get(contact.key.number)?.color || hitColor);
      contact.plume.visible = contact.key.glow > .015;
    }

    if (!reduceMotion.matches && playback.playing) {
      for (const [number, event] of active) if (Math.random() < dt * 14) emit(whiteKeys.get(number), event.color, false);
    }
    let count = 0;
    for (let i = particles.length - 1; i >= 0; i--) {
      const p = particles[i];
      p.age += dt;
      if (p.age >= p.life || reduceMotion.matches) { particles.splice(i, 1); continue; }
      p.x += (p.vx + Math.sin(p.age * 4 + p.z) * .5) * dt;
      p.y += p.vy * dt;
      p.vx *= Math.exp(-dt * 1.4);
      p.z += dt * .12;
      particlePositions.set([p.x, p.y, p.z], count * 3);
      particleColors.set([p.color.r * 2, p.color.g * 2, p.color.b * 2], count * 3);
      particleSizes[count] = p.size * (p.fog ? 1 + p.age : 1);
      particleAlpha[count] = (1 - p.age / p.life) * (p.fog ? .055 : .8);
      count++;
    }
    particleGeometry.setDrawRange(0, count);
    for (const attribute of Object.values(particleGeometry.attributes)) attribute.needsUpdate = true;
    const labels = [...active.values()].map(({ note }) => ['C', 'C♯', 'D', 'D♯', 'E', 'F', 'F♯', 'G', 'G♯', 'A', 'A♯', 'B'][note.midi % 12] + (Math.floor(note.midi / 12) - 1)).join('  ·  ');
    if (labels !== lastLabels) {
      document.getElementById('noteReadout').textContent = labels;
      lastLabels = labels;
    }
    composite();
    frameCount++;
  }
  renderer.setAnimationLoop(render);

  renderer.domElement.addEventListener('pointerdown', event => {
    if (camera.aspect >= .85 || readPlayback().playing) return;
    dragState = { x: event.clientX, center: viewCenter, moved: false };
    renderer.domElement.setPointerCapture(event.pointerId);
  });
  renderer.domElement.addEventListener('pointermove', event => {
    if (!dragState) return;
    const dx = event.clientX - dragState.x;
    if (Math.abs(dx) < 7 && !dragState.moved) return;
    dragState.moved = true;
    manualPan = true;
    viewCenter = THREE.MathUtils.clamp(dragState.center - dx / width * 17, -panLimit, panLimit);
    resize();
  });
  renderer.domElement.addEventListener('pointercancel', () => { dragState = null; });
  renderer.domElement.addEventListener('pointerup', event => {
    const dragged = dragState?.moved;
    dragState = null;
    if (dragged) return;
    const state = readPlayback();
    if (state.hasRun || state.loading) return;
    const bounds = renderer.domElement.getBoundingClientRect();
    pointer.set((event.clientX - bounds.left) / bounds.width * 2 - 1, 1 - (event.clientY - bounds.top) / bounds.height * 2);
    raycaster.setFromCamera(pointer, camera);
    const hit = raycaster.intersectObjects(keys.map(key => key.mesh))[0];
    if (!hit) return;
    const key = hit.object.userData.key;
    if (key.isBlack) return;
    key.manualUntil = performance.now() + 400;
    if (!reduceMotion.matches) emit(key, hitColor, true);
    audition(key.number);
  });
  renderer.domElement.addEventListener('webglcontextlost', event => {
    event.preventDefault();
    renderer.setAnimationLoop(null);
    document.getElementById('notice').textContent = '三维图形设备中断，请刷新页面后重试。';
  });
  window.addEventListener('pagehide', () => {
    renderer.setAnimationLoop(null);
    resizeObserver.disconnect();
    renderer.dispose();
    sceneTarget.dispose(); bloomA.dispose(); bloomB.dispose();
  }, { once: true });
  return { get frameCount() { return frameCount; }, get visibleNotes() { return notes.count; }, get activeKeys() { return keys.filter(k => k.press > .5).map(k => k.midi); } };
}
