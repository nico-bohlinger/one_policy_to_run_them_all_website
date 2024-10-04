import * as THREE from 'three';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { setupGUI, loadSceneFromURL, downloadRobotsFolder, getPosition, getQuaternion } from './mujocoUtils.js';
import { create_robot_action_handlers } from './robot_action_handlers.js';
import { default_camera_position, default_camera_target } from './camera_consts.js';
import load_mujoco from "../mujoco_wasm/mujoco_wasm.js";

// Load the MuJoCo Module
const mujoco = await load_mujoco();

var initialScene = "unitree_h1/unitree_h1.xml";

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
await downloadRobotsFolder(mujoco);
var robot_action_handlers = create_robot_action_handlers();


export class MuJoCoApp {
    constructor() {
        this.mujoco = mujoco;

        this.model = new mujoco.Model("/working/" + initialScene);
        this.state = new mujoco.State(this.model);
        this.simulation = new mujoco.Simulation(this.model, this.state);

        this.params = { scene: initialScene, paused: false, free_camera: true, x_goal_velocity: 0.7, y_goal_velocity: 0.0, yaw_goal_velocity: 0.0 };
        this.mujoco_time = 0.0;
        this.bodies  = {}, this.lights = {};
        this.tmpVec  = new THREE.Vector3();
        this.tmpQuat = new THREE.Quaternion();
        this.updateGUICallbacks = [];

        this.container = document.getElementById('canvas-container');

        this.scene = new THREE.Scene();
        this.scene.name = 'scene';

        var height = 500;
        var width = Math.min(800, window.innerWidth);
        console.log(width, height);

        this.camera = new THREE.PerspectiveCamera( 45, width / height, 0.001, 100 );
        this.camera.name = 'PerspectiveCamera';
        this.camera.position.set(default_camera_position.x, default_camera_position.y, default_camera_position.z);
        this.scene.add(this.camera);

        this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
        this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

        this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
        this.ambientLight.name = 'AmbientLight';
        this.scene.add( this.ambientLight );

        this.renderer = new THREE.WebGLRenderer( { antialias: true } );
        this.renderer.setPixelRatio( window.devicePixelRatio );
        this.renderer.setSize(width, height);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
        this.renderer.setAnimationLoop( this.render.bind(this) );

        this.container.innerHTML = "";
        this.container.appendChild( this.renderer.domElement );

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.target.set(default_camera_target.x, default_camera_target.y, default_camera_target.z);
        this.controls.panSpeed = 2;
        this.controls.zoomSpeed = 1;
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.10;
        this.controls.screenSpacePanning = true;
        this.controls.update();

        this.previous_scene = initialScene;
        this.qpos_history = Array(100).fill().map(() => Array(3).fill(0));
        this.previously_free_camera = this.params["free_camera"];

        window.addEventListener('resize', this.onWindowResize.bind(this));
    }
  
    async init() {
        [this.model, this.state, this.simulation, this.bodies, this.lights] = await loadSceneFromURL(mujoco, initialScene, this);
        this.gui = new GUI();
        this.container.appendChild(this.gui.domElement);
        setupGUI(this);
    }
  
    onWindowResize() {
        var height = 500;
        var width = Math.min(800, window.innerWidth);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }
  
    render(timeMS) {
        this.controls.update();

        if (!this.params["paused"]) {
            if (!this.params["free_camera"]) {
                let x_pos = this.simulation.qpos[0];
                let y_pos = this.simulation.qpos[1];
                let z_pos = this.simulation.qpos[2];
                if (!this.previously_free_camera) {
                    this.qpos_history.shift();
                    this.qpos_history.push([x_pos, y_pos, z_pos]);
                } else {
                    this.qpos_history = Array(100).fill().map(() => [x_pos, y_pos, z_pos]);
                }
                let avg_x_pos = this.qpos_history.reduce((a, b) => a + b[0], 0) / this.qpos_history.length;
                let avg_y_pos = this.qpos_history.reduce((a, b) => a + b[1], 0) / this.qpos_history.length;
                let avg_z_pos = this.qpos_history.reduce((a, b) => a + b[2], 0) / this.qpos_history.length;
                this.camera.position.x = avg_x_pos
                this.camera.position.y = default_camera_position.y + avg_z_pos
                this.camera.position.z = default_camera_position.z - avg_y_pos
                this.controls.target.x = avg_x_pos
                this.controls.target.y = default_camera_target.y + avg_z_pos
                this.controls.target.z = default_camera_target.z - avg_y_pos
                this.controls.update();
            }

            let scene = this.params["scene"];
            let robot_action_handler = robot_action_handlers[scene];

            if (!robot_action_handler.is_initialized) {
                robot_action_handler.init(this.model);
            }

            if (scene != this.previous_scene) {
                robot_action_handler.reset();
                this.previous_scene = scene;
                this.qpos_history = Array(100).fill().map(() => Array(3).fill(0));
            }

            tf.tidy(() => {
                robot_action_handler.action(this.simulation, this.model, this.params["x_goal_velocity"], this.params["y_goal_velocity"], this.params["yaw_goal_velocity"]);
            });

            let timestep = this.model.getOptions().timestep * 4;
            if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
            while (this.mujoco_time < timeMS) {
                this.simulation.step();
                this.simulation.step();
                this.simulation.step();
                this.simulation.step();
                this.mujoco_time += timestep * 1000.0;
            }
        }

        // Update body transforms.
        for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
                getPosition  (this.simulation.xpos , b, this.bodies[b].position);
                getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
                this.bodies[b].updateWorldMatrix();
            }
        }
    
        // Update light transforms.
        for (let l = 0; l < this.model.nlight; l++) {
            if (this.lights[l]) {
                getPosition(this.simulation.light_xpos, l, this.lights[l].position);
                getPosition(this.simulation.light_xdir, l, this.tmpVec);
                this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
            }
        }
    
        // Update tendon transforms.
        let numWraps = 0;
        if (this.mujocoRoot && this.mujocoRoot.cylinders) {
            let mat = new THREE.Matrix4();
            for (let t = 0; t < this.model.ntendon; t++) {
            let startW = this.simulation.ten_wrapadr[t];
            let r = this.model.tendon_width[t];
            for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] -1 ; w++) {
                let tendonStart = getPosition(this.simulation.wrap_xpos, w    , new THREE.Vector3());
                let tendonEnd   = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
                let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);
    
                let validStart = tendonStart.length() > 0.01;
                let validEnd   = tendonEnd  .length() > 0.01;
    
                if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
                if (validEnd  ) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
                if (validStart && validEnd) {
                    mat.compose(tendonAvg, new THREE.Quaternion().setFromUnitVectors(
                        new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
                        new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
                    this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
                    numWraps++;
                }
            }
            }
            this.mujocoRoot.cylinders.count = numWraps;
            this.mujocoRoot.spheres  .count = numWraps > 0 ? numWraps + 1: 0;
            this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
            this.mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
        }

        this.renderer.render(this.scene, this.camera);
    }
  }


let app = new MuJoCoApp();
await app.init();
