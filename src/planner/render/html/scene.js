
// setup scene
let renderer = new THREE.WebGLRenderer();
document.body.appendChild(renderer.domElement);

var scene = new THREE.Scene();

// prepare environment for dull reflections
const pmremGenerator = new THREE.PMREMGenerator(renderer);
pmremGenerator.compileCubemapShader()

const loader = new THREE.CubeTextureLoader();
textureCube = loader.load([
	`environment/px.jpg`, `environment/nx.jpg`,
	`environment/py.jpg`, `environment/ny.jpg`,
	`environment/pz.jpg`, `environment/nz.jpg`,
], function (texture) {
	let pmrem = pmremGenerator.fromCubemap(texture)
	scene.environment = pmrem.texture;
});

// material for collision hulls without margin
const passiveHullMaterial = new THREE.MeshStandardMaterial({
	color: 0x11aabb,
	envMapIntensity: 0.7,
	roughness: 0.15,
	metalness: 0.3
});

const activeHullMaterial = new THREE.MeshStandardMaterial({
	color: 0xffbb11,
	envMapIntensity: 0.7,
	roughness: 0.15,
	metalness: 0.3
});

const passiveCollidingHullMaterial = new THREE.MeshStandardMaterial({
	color: 0xaa0000,
	emissive: 0x200000,
	envMapIntensity: 0.7,
	roughness: 0.15,
	metalness: 0.3
});

const activeCollidingHullMaterial = new THREE.MeshStandardMaterial({
	color: 0xff5000,
	emissive: 0x401000,
	envMapIntensity: 0.7,
	roughness: 0.15,
	metalness: 0.3
});


// material for collision probing result
const probeMaterial = new THREE.MeshBasicMaterial({
	color: 0x88ddff,
	wireframe: true
});

// material for the icosahedron in the background
const backgroundMaterial = new THREE.MeshStandardMaterial({
	color: 0xddddee,
	emissive: 0xc2c2c2,
	envMapIntensity: 0.5,
	roughness: 0.3,
	metalness: 0.9,
	side: THREE.BackSide,
	depthTest: false,
	depthWrite: false,
});

// material for collision margin
const marginVertexShader = `
varying vec3 nml;
varying vec3 pos;

void main()
{
	pos = (modelMatrix * vec4(position, 1.0)).xyz;
	nml = (modelMatrix * vec4(normal, 0.0)).xyz;
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}
`
const marginFragmentShader = `
uniform vec3 glowColor;
varying vec3 pos;
varying vec3 nml;
void main()
{
	vec3 view = normalize(pos-cameraPosition);
	float intensity = pow(1.0 - abs(dot(normalize(nml), view)), 3.0);
	gl_FragColor = vec4( glowColor, 0.7 * intensity + 0.3 );
}
`

var passiveMarginMaterial = new THREE.ShaderMaterial({
	uniforms: {
		glowColor: { value: new THREE.Color(0x880088) },
	},
	vertexShader: marginVertexShader,
	fragmentShader: marginFragmentShader,
	side: THREE.BackSide,
	//blending: THREE.AdditiveBlending,
	transparent: true,
	//depthWrite: false
});

var activeMarginMaterial = new THREE.ShaderMaterial({
	uniforms: {
		glowColor: { value: new THREE.Color(0x11eeff) },
	},
	vertexShader: marginVertexShader,
	fragmentShader: marginFragmentShader,
	side: THREE.BackSide,
	//blending: THREE.AdditiveBlending,
	transparent: true,
	//depthWrite: false
});

var passiveCollidingMarginMaterial = new THREE.ShaderMaterial({
	uniforms: {
		glowColor: { value: new THREE.Color(0x000000) },
	},
	vertexShader: marginVertexShader,
	fragmentShader: marginFragmentShader,
	side: THREE.BackSide,
	//blending: THREE.AdditiveBlending,
	transparent: true,
	//depthWrite: false
});

var activeCollidingMarginMaterial = new THREE.ShaderMaterial({
	uniforms: {
		glowColor: { value: new THREE.Color(0xee0000) },
	},
	vertexShader: marginVertexShader,
	fragmentShader: marginFragmentShader,
	side: THREE.BackSide,
	//blending: THREE.AdditiveBlending,
	transparent: true,
	//depthWrite: false
});

// camera and mouse control
var camera = new THREE.PerspectiveCamera(45, 0.75, 0.01, 100);
camera.position.set(1, 1, 1);
camera.lookAt(new THREE.Vector3(0, 0, 0));
camera.up.set(0, 0, 1);
var controls = new THREE.OrbitControls(camera, renderer.domElement);

// lighting
const light1 = new THREE.HemisphereLight(0xdddddd, 0xaaaaaa, 1);
light1.position.set(1.3, 1.2, 1.1);
scene.add(light1);
const light2 = new THREE.DirectionalLight(0x333333, 1);
light2.position.set(1.5, 1.3, 1.1);
scene.add(light2);
const light3 = new THREE.PointLight(0x111111);
scene.add(light3);

// background icosahedron
const icoGeometry = new THREE.IcosahedronGeometry(20)
const ico = new THREE.Mesh(icoGeometry, backgroundMaterial)
ico.renderOrder = -1
scene.add(ico);

// visualization for collisions
const collisionSparkGeometry = new THREE.BufferGeometry();
collisionSparkGeometry.setAttribute(
	'position', new THREE.Float32BufferAttribute(
		[0, 0, 0, 0.15, 0, 0, 0.075, 0.01, 0], 3)
);
collisionSparkGeometry.setIndex([0, 1, 2]);
collisionSparkGeometry.computeVertexNormals();

// coordinate system
const axesHelper = new THREE.AxesHelper(1);
//scene.add(axesHelper);

// canvas resizes with window
window.addEventListener('resize', onWindowResize, false);
function onWindowResize() {
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(window.innerWidth, window.innerHeight);
}
onWindowResize()

// DRY helper
function transform(obj, trafo) {
	obj.position.set(trafo[0], trafo[1], trafo[2])
	obj.quaternion.set(trafo[3], trafo[4], trafo[5], trafo[6])
}

// generate all meshes
let geometries = {};
for (const hash in sceneData.meshes) {
	const data = sceneData.meshes[hash];
	let geometry = new THREE.BufferGeometry();
	geometry.setAttribute('position',
		new THREE.Float32BufferAttribute(data.vertices.flat(), 3));
	geometry.setIndex(data.faces.flat());
	geometry.computeVertexNormals();
	geometries[hash] = geometry;
}

let drawCollisionFireworks = false;

// draw collision
function collision(col) {
	if (drawCollisionFireworks) {
		for (let i = 0; i < 20; i++) {
			const color = new THREE.Color(1, Math.random() * 0.6, 0)
			const ori = new THREE.Euler(
				Math.random() * 10000, Math.random() * 10000, Math.random() * 10000)

			for (let edges of [false, true]) {

				const collisionSparkMaterial = new THREE.MeshBasicMaterial({
					color: color,
					side: THREE.DoubleSide,
					wireframe: edges,
					depthTest: !edges
				});
				const mesh = new THREE.Mesh(collisionSparkGeometry, collisionSparkMaterial);
				mesh.name = "collision"
				if (edges) {
					mesh.renderOrder = 1;
				}
				scene.add(mesh);
				mesh.position.set(col.xyz[0], col.xyz[1], col.xyz[2]);
				mesh.setRotationFromEuler(ori)
			}
		}
	}
	hull1 = scene.getObjectByName(col.obj1 + ".hull")
	margin1 = scene.getObjectByName(col.obj1 + ".margin")
	hull1.material = hull1.userData.isActuated ? activeCollidingHullMaterial : passiveCollidingHullMaterial
	margin1.material = margin1.userData.isActuated ? activeCollidingMarginMaterial : passiveCollidingMarginMaterial
	hull2 = scene.getObjectByName(col.obj2 + ".hull")
	margin2 = scene.getObjectByName(col.obj2 + ".margin")
	hull2.material = hull2.userData.isActuated ? activeCollidingHullMaterial : passiveCollidingHullMaterial
	margin2.material = margin2.userData.isActuated ? activeCollidingMarginMaterial : passiveCollidingMarginMaterial

	console.log("collision: " + col.obj1 + " + " + col.obj2)
}

let player = null;
let play;
function pause() {
	if (player != null) {
		clearInterval(player)
		player = null
	}
}
function togglePlay() {
	if (player == null) {
		play()
	}
	else {
		pause()
	}
}

function drawScene(data) {
	document.title = data.title + " - GestaltPlanner"

	for (const name in data.objects) {
		let obj = data.objects[name]
		let geometry = geometries[obj.mesh].clone()

		let material
		if (obj.meshType == "hull") {
			geometry = geometry.toNonIndexed();
			geometry.computeVertexNormals();
			material = obj.isActuated ?
				activeHullMaterial : passiveHullMaterial
		}
		else if (obj.meshType == "margin") {
			material = obj.isActuated ?
				activeMarginMaterial : passiveMarginMaterial
		}

		let mesh = new THREE.Mesh(geometry, material)
		mesh.name = name
		mesh.userData = {
			isActuated: obj.isActuated,
			meshType: obj.meshType,
			defaultMaterial: material
		}
		transform(mesh, obj.isActuated ? obj.xyz_qxyzw[0] : obj.xyz_qxyzw)
		scene.add(mesh)
	}

	let restoreMaterials = function () {
		for (const mesh of scene.children) {
			if ("defaultMaterial" in mesh.userData) {
				mesh.material = mesh.userData.defaultMaterial
			}
		}
	}

	let frame = 0;

	function redrawCollisions() {
		restoreMaterials()
		while (true) {
			let col = scene.getObjectByName("collision")
			if (col) { scene.remove(col) }
			else { break }
		}
		if (frame in data.collisions) {
			for (let col of data.collisions[frame]) {
				collision(col)
			}
		}
	}
	redrawCollisions()

	document.addEventListener('keydown', function (event) {
		switch (event.key) {
			case "x":
				camera.up.set(1, 0, 0);
				break;
			case "y":
				camera.up.set(0, 1, 0);
				break;
			case "z":
				camera.up.set(0, 0, 1);
				break;
			case "c":
				drawCollisionFireworks = !drawCollisionFireworks;
				redrawCollisions()
		}
	}, false);

	if (data.numSteps > 0) {

		let compose = function () {
			for (const name in data.objects) {
				let obj = data.objects[name]
				if (obj.isActuated) {
					transform(scene.getObjectByName(name), obj.xyz_qxyzw[frame])
				}
			}
			redrawCollisions()
		}

		let autoplay = true;

		document.addEventListener('keydown', function (event) {
			autoplay = false;
			switch (event.key) {
				case " ":
					if (frame == data.numSteps - 1) { frame = 0 }
					togglePlay()
					break;
				case "ArrowLeft":
					pause()
					if (frame > 0) { frame-- }
					compose()
					break;
				case "ArrowRight":
					pause()
					if (frame < data.numSteps - 1) { frame++ }
					else { pause() }
					compose()
					break;
				case "ArrowUp":
					pause()
					frame = 0
					compose()
					break;
				case "ArrowDown":
					pause()
					frame = data.numSteps - 1
					compose()
					break;
			}
		}, false);

		play = () => {
			player = setInterval(() => {
				if (frame < data.numSteps - 1) {
					frame++
					compose()
				}
				else {
					pause()
				}
			}, data.dt * 1000)
		}

		setTimeout(() => {
			if (autoplay) { play() }
		}, 2000)
	}
}

drawScene(sceneData.scenes[0])

// main loop
function render() {
	requestAnimationFrame(render);
	controls.update();
	light3.position.copy(camera.position)
	ico.position.copy(camera.position)
	renderer.render(scene, camera);
}
render();
