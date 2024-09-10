import * as THREE from 'three';
import { Reflector  } from './Reflector.js';


export async function reloadFunc() {
  // Delete the old scene and load the new scene
  this.scene.remove(this.scene.getObjectByName("MuJoCo Root"));
  [this.model, this.state, this.simulation, this.bodies, this.lights] =
    await loadSceneFromURL(this.mujoco, this.params.scene, this);
  this.simulation.qpos.set(this.model.key_qpos);
  this.simulation.forward();
  for (let i = 0; i < this.updateGUICallbacks.length; i++) {
    this.updateGUICallbacks[i](this.model, this.simulation, this.params);
  }
}

export function setupGUI(parentContext) {

  // Make sure we reset the camera when the scene is changed or reloaded.
  parentContext.updateGUICallbacks.length = 0;
  parentContext.updateGUICallbacks.push((model, simulation, params) => {
  // TODO: Use free camera parameters from MuJoCo
  parentContext.camera.position.set(2.0, 1.7, 1.7);
  parentContext.controls.target.set(0, 0.7, 0);
  parentContext.controls.update(); });

  // Add scene selection dropdown.
  let reload = reloadFunc.bind(parentContext);
  parentContext.gui.add(parentContext.params, 'scene', {
    "Unitree A1": "unitree_a1/unitree_a1.xml",
    "Unitree Go1": "unitree_go1/unitree_go1.xml",
    "Unitree Go2": "unitree_go2/unitree_go2.xml",
    "ANYbotics ANYmal B": "anybotics_anymal_b/anybotics_anymal_b.xml",
    "ANYbotics ANYmal C": "anybotics_anymal_c/anybotics_anymal_c.xml",
    "Google Barkour v0": "google_barkour_v0/google_barkour_v0.xml",
    "Google Barkour vB": "google_barkour_vb/google_barkour_vb.xml",
    "MAB Silver Badger": "mab_silver_badger/mab_silver_badger.xml",
    "Petoi Bittle": "petoi_bittle/petoi_bittle.xml",
    "Unitree H1": "unitree_h1/unitree_h1.xml",
    "Unitree G1": "unitree_g1/unitree_g1.xml",
    "PAL Robotics Talos": "pal_robotics_talos/pal_robotics_talos.xml",
    "Robotis OP3": "robotis_op3/robotis_op3.xml",
    "SoftBank Nao V5": "softbank_nao_v5/softbank_nao_v5.xml",
    "Agility Robotics Cassie": "agility_robotics_cassie/agility_robotics_cassie.xml",
    "Custom Hexapod": "hexapod/hexapod.xml",
  }).name('Robot').onChange(reload);

  // Add pause simulation checkbox.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Pause Simulation".
  //  When paused, a "pause" text in white is displayed in the top left corner.
  //  Can also be triggered by pressing the spacebar.
  const pauseSimulation = parentContext.gui.add(parentContext.params, 'paused').name('Pause Simulation');
  pauseSimulation.onChange((value) => {
    if (value) {
      const pausedText = document.createElement('div');
      pausedText.style.position = 'absolute';
      pausedText.style.top = '10px';
      pausedText.style.left = '10px';
      pausedText.style.color = 'white';
      pausedText.style.font = 'normal 18px Arial';
      pausedText.innerHTML = 'pause';
      parentContext.container.appendChild(pausedText);
    } else {
      parentContext.container.removeChild(parentContext.container.lastChild);
    }
  });
  document.addEventListener('keydown', (event) => {
    if (event.code === 'Space') {
      parentContext.params.paused = !parentContext.params.paused;
      pauseSimulation.setValue(parentContext.params.paused);
      event.preventDefault();
    }
  });

  // Add reset simulation button.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Reset".
  //  When pressed, resets the simulation to the initial state.
  //  Can also be triggered by pressing backspace.
  const resetSimulation = () => {
    parentContext.simulation.resetData();
    parentContext.simulation.qpos.set(parentContext.model.key_qpos);
    parentContext.simulation.forward();
  };
  parentContext.gui.add({reset: () => { resetSimulation(); }}, 'reset').name('Reset');
  document.addEventListener('keydown', (event) => {
    if (event.code === 'Backspace') { resetSimulation(); event.preventDefault(); }});

  parentContext.gui.open();
}

export async function loadSceneFromURL(mujoco, filename, parent) {
    // Free the old simulation.
    if (parent.simulation != null) {
      parent.simulation.free();
      parent.model      = null;
      parent.state      = null;
      parent.simulation = null;
    }

    // Load in the state from XML.
    parent.model       = mujoco.Model.load_from_xml("/working/"+filename);
    parent.state       = new mujoco.State(parent.model);
    parent.simulation  = new mujoco.Simulation(parent.model, parent.state);

    let model = parent.model;
    let state = parent.state;
    let simulation = parent.simulation;

    // Decode the null-terminated string names.
    let textDecoder = new TextDecoder("utf-8");
    let fullString = textDecoder.decode(model.names);
    let names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));

    // Create the root object.
    let mujocoRoot = new THREE.Group();
    mujocoRoot.name = "MuJoCo Root"
    parent.scene.add(mujocoRoot);

    /** @type {Object.<number, THREE.Group>} */
    let bodies = {};
    /** @type {Object.<number, THREE.BufferGeometry>} */
    let meshes = {};
    /** @type {THREE.Light[]} */
    let lights = [];

    // Default material definition.
    let material = new THREE.MeshPhysicalMaterial();
    material.color = new THREE.Color(1, 1, 1);

    // Loop through the MuJoCo geoms and recreate them in three.js.
    for (let g = 0; g < model.ngeom; g++) {
      // Only visualize geom groups up to 2 (same default behavior as simulate).
      if (!(model.geom_group[g] < 3)) { continue; }

      // Get the body ID and type of the geom.
      let b = model.geom_bodyid[g];
      let type = model.geom_type[g];
      let size = [
        model.geom_size[(g*3) + 0],
        model.geom_size[(g*3) + 1],
        model.geom_size[(g*3) + 2]
      ];

      // Create the body if it doesn't exist.
      if (!(b in bodies)) {
        bodies[b] = new THREE.Group();
        bodies[b].name = names[model.name_bodyadr[b]];
        bodies[b].bodyID = b;
        bodies[b].has_custom_mesh = false;
      }

      // Set the default geometry. In MuJoCo, this is a sphere.
      let geometry = new THREE.SphereGeometry(size[0] * 0.5);
      if (type == mujoco.mjtGeom.mjGEOM_PLANE.value) {
        // Special handling for plane later.
      } else if (type == mujoco.mjtGeom.mjGEOM_HFIELD.value) {
        // TODO: Implement this.
      } else if (type == mujoco.mjtGeom.mjGEOM_SPHERE.value) {
        geometry = new THREE.SphereGeometry(size[0]);
      } else if (type == mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
        geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
      } else if (type == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
        geometry = new THREE.SphereGeometry(1); // Stretch this below
      } else if (type == mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
        geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
      } else if (type == mujoco.mjtGeom.mjGEOM_BOX.value) {
        geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
      } else if (type == mujoco.mjtGeom.mjGEOM_MESH.value) {
        let meshID = model.geom_dataid[g];

        if (!(meshID in meshes)) {
          geometry = new THREE.BufferGeometry(); // TODO: Populate the Buffer Geometry with Generic Mesh Data

          let vertex_buffer = model.mesh_vert.subarray(
             model.mesh_vertadr[meshID] * 3,
            (model.mesh_vertadr[meshID]  + model.mesh_vertnum[meshID]) * 3);
          for (let v = 0; v < vertex_buffer.length; v+=3){
            //vertex_buffer[v + 0] =  vertex_buffer[v + 0];
            let temp             =  vertex_buffer[v + 1];
            vertex_buffer[v + 1] =  vertex_buffer[v + 2];
            vertex_buffer[v + 2] = -temp;
          }

          let normal_buffer = model.mesh_normal.subarray(
             model.mesh_vertadr[meshID] * 3,
            (model.mesh_vertadr[meshID]  + model.mesh_vertnum[meshID]) * 3);
          for (let v = 0; v < normal_buffer.length; v+=3){
            //normal_buffer[v + 0] =  normal_buffer[v + 0];
            let temp             =  normal_buffer[v + 1];
            normal_buffer[v + 1] =  normal_buffer[v + 2];
            normal_buffer[v + 2] = -temp;
          }

          let uv_buffer = model.mesh_texcoord.subarray(
             model.mesh_texcoordadr[meshID] * 2,
            (model.mesh_texcoordadr[meshID]  + model.mesh_vertnum[meshID]) * 2);
          let triangle_buffer = model.mesh_face.subarray(
             model.mesh_faceadr[meshID] * 3,
            (model.mesh_faceadr[meshID]  + model.mesh_facenum[meshID]) * 3);
          geometry.setAttribute("position", new THREE.BufferAttribute(vertex_buffer, 3));
          geometry.setAttribute("normal"  , new THREE.BufferAttribute(normal_buffer, 3));
          geometry.setAttribute("uv"      , new THREE.BufferAttribute(    uv_buffer, 2));
          geometry.setIndex    (Array.from(triangle_buffer));
          meshes[meshID] = geometry;
        } else {
          geometry = meshes[meshID];
        }

        bodies[b].has_custom_mesh = true;
      }
      // Done with geometry creation.

      // Set the Material Properties of incoming bodies
      let texture = undefined;
      let color = [
        model.geom_rgba[(g * 4) + 0],
        model.geom_rgba[(g * 4) + 1],
        model.geom_rgba[(g * 4) + 2],
        model.geom_rgba[(g * 4) + 3]];
      if (model.geom_matid[g] != -1) {
        let matId = model.geom_matid[g];
        color = [
          model.mat_rgba[(matId * 4) + 0],
          model.mat_rgba[(matId * 4) + 1],
          model.mat_rgba[(matId * 4) + 2],
          model.mat_rgba[(matId * 4) + 3]];

        // Construct Texture from model.tex_rgb
        texture = undefined;
        let texId = model.mat_texid[matId];
        if (texId != -1) {
          let width    = model.tex_width [texId];
          let height   = model.tex_height[texId];
          let offset   = model.tex_adr   [texId];
          let rgbArray = model.tex_rgb   ;
          let rgbaArray = new Uint8Array(width * height * 4);
          for (let p = 0; p < width * height; p++){
            rgbaArray[(p * 4) + 0] = rgbArray[offset + ((p * 3) + 0)];
            rgbaArray[(p * 4) + 1] = rgbArray[offset + ((p * 3) + 1)];
            rgbaArray[(p * 4) + 2] = rgbArray[offset + ((p * 3) + 2)];
            rgbaArray[(p * 4) + 3] = 1.0;
          }
          texture = new THREE.DataTexture(rgbaArray, width, height, THREE.RGBAFormat, THREE.UnsignedByteType);
          if (texId == 2) {
            texture.repeat = new THREE.Vector2(50, 50);
            texture.wrapS = THREE.RepeatWrapping;
            texture.wrapT = THREE.RepeatWrapping;
          } else {
            texture.repeat = new THREE.Vector2(1, 1);
            texture.wrapS = THREE.RepeatWrapping;
            texture.wrapT = THREE.RepeatWrapping;
          }

          texture.needsUpdate = true;
        }
      }

      if (material.color.r != color[0] ||
          material.color.g != color[1] ||
          material.color.b != color[2] ||
          material.opacity != color[3] ||
          material.map     != texture) {
        material = new THREE.MeshPhysicalMaterial({
          color: new THREE.Color(color[0], color[1], color[2]),
          transparent: color[3] < 1.0,
          opacity: color[3],
          specularIntensity: model.geom_matid[g] != -1 ?       model.mat_specular   [model.geom_matid[g]] *0.5 : undefined,
          reflectivity     : model.geom_matid[g] != -1 ?       model.mat_reflectance[model.geom_matid[g]] : undefined,
          roughness        : model.geom_matid[g] != -1 ? 1.0 - model.mat_shininess  [model.geom_matid[g]] : undefined,
          metalness        : model.geom_matid[g] != -1 ? 0.1 : undefined,
          map              : texture
        });
      }

      let mesh = new THREE.Mesh();
      if (type == 0) {
        mesh = new Reflector( new THREE.PlaneGeometry( 100, 100 ), { clipBias: 0.003,texture: texture } );
        mesh.rotateX( - Math.PI / 2 );
      } else {
        mesh = new THREE.Mesh(geometry, material);
      }

      mesh.castShadow = g == 0 ? false : true;
      mesh.receiveShadow = type != 7;
      mesh.bodyID = b;
      bodies[b].add(mesh);
      getPosition  (model.geom_pos, g, mesh.position  );
      if (type != 0) { getQuaternion(model.geom_quat, g, mesh.quaternion); }
      if (type == 4) { mesh.scale.set(size[0], size[2], size[1]) } // Stretch the Ellipsoid
    }

    // Parse tendons.
    let tendonMat = new THREE.MeshPhongMaterial();
    tendonMat.color = new THREE.Color(0.8, 0.3, 0.3);
    mujocoRoot.cylinders = new THREE.InstancedMesh(
        new THREE.CylinderGeometry(1, 1, 1),
        tendonMat, 1023);
    mujocoRoot.cylinders.receiveShadow = true;
    mujocoRoot.cylinders.castShadow    = true;
    mujocoRoot.add(mujocoRoot.cylinders);
    mujocoRoot.spheres = new THREE.InstancedMesh(
        new THREE.SphereGeometry(1, 10, 10),
        tendonMat, 1023);
    mujocoRoot.spheres.receiveShadow = true;
    mujocoRoot.spheres.castShadow    = true;
    mujocoRoot.add(mujocoRoot.spheres);

    // Parse lights.
    for (let l = 0; l < model.nlight; l++) {
      let light = new THREE.SpotLight();
      if (model.light_directional[l]) {
        light = new THREE.DirectionalLight();
      } else {
        light = new THREE.SpotLight();
      }
      light.decay = model.light_attenuation[l] * 100;
      light.penumbra = 0.5;
      light.castShadow = true; // default false

      light.shadow.mapSize.width = 1024; // default
      light.shadow.mapSize.height = 1024; // default
      light.shadow.camera.near = 1; // default
      light.shadow.camera.far = 10; // default
      //bodies[model.light_bodyid()].add(light);
      if (bodies[0]) {
        bodies[0].add(light);
      } else {
        mujocoRoot.add(light);
      }
      lights.push(light);
    }
    if (model.nlight == 0) {
      let light = new THREE.DirectionalLight();
      mujocoRoot.add(light);
    }

    for (let b = 0; b < model.nbody; b++) {
      //let parent_body = model.body_parentid()[b];
      if (b == 0 || !bodies[0]) {
        mujocoRoot.add(bodies[b]);
      } else if(bodies[b]){
        bodies[0].add(bodies[b]);
      } else {
        console.log("Body without Geometry detected; adding to bodies", b, bodies[b]);
        bodies[b] = new THREE.Group(); bodies[b].name = names[b + 1]; bodies[b].bodyID = b; bodies[b].has_custom_mesh = false;
        bodies[0].add(bodies[b]);
      }
    }
  
    parent.mujocoRoot = mujocoRoot;

    simulation.qpos.set(model.key_qpos);

    return [model, state, simulation, bodies, lights]
}

export async function downloadRobotsFolder(mujoco) {
  let allFiles = [
    "agility_robotics_cassie/agility_robotics_cassie.xml",
    "agility_robotics_cassie/assets/achilles-rod.obj",
    "agility_robotics_cassie/assets/cassie-texture.png",
    "agility_robotics_cassie/assets/foot-crank.obj",
    "agility_robotics_cassie/assets/foot.obj",
    "agility_robotics_cassie/assets/heel-spring.obj",
    "agility_robotics_cassie/assets/hip-pitch.obj",
    "agility_robotics_cassie/assets/hip-roll.obj",
    "agility_robotics_cassie/assets/hip-yaw.obj",
    "agility_robotics_cassie/assets/knee-spring.obj",
    "agility_robotics_cassie/assets/knee.obj",
    "agility_robotics_cassie/assets/pelvis.obj",
    "agility_robotics_cassie/assets/plantar-rod.obj",
    "agility_robotics_cassie/assets/shin.obj",
    "agility_robotics_cassie/assets/tarsus.obj",
    "anybotics_anymal_b/anybotics_anymal_b.xml",
    "anybotics_anymal_b/assets/anymal_base_0.obj",
    "anybotics_anymal_b/assets/anymal_base_1.obj",
    "anybotics_anymal_b/assets/anymal_base_2.obj",
    "anybotics_anymal_b/assets/anymal_base_3.obj",
    "anybotics_anymal_b/assets/anymal_base_4.obj",
    "anybotics_anymal_b/assets/anymal_base_5.obj",
    "anybotics_anymal_b/assets/anymal_base_6.obj",
    "anybotics_anymal_b/assets/anymal_base_7.obj",
    "anybotics_anymal_b/assets/anymal_foot_0.obj",
    "anybotics_anymal_b/assets/anymal_foot_1.obj",
    "anybotics_anymal_b/assets/anymal_foot_2.obj",
    "anybotics_anymal_b/assets/anymal_foot_3.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_0.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_1.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_2.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_3.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_4.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_5.obj",
    "anybotics_anymal_b/assets/anymal_hip_l_6.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_0.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_1.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_2.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_3.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_4.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_5.obj",
    "anybotics_anymal_b/assets/anymal_hip_r_6.obj",
    "anybotics_anymal_b/assets/anymal_shank_l_0.obj",
    "anybotics_anymal_b/assets/anymal_shank_l_1.obj",
    "anybotics_anymal_b/assets/anymal_shank_l_2.obj",
    "anybotics_anymal_b/assets/anymal_shank_l_3.obj",
    "anybotics_anymal_b/assets/anymal_shank_r_0.obj",
    "anybotics_anymal_b/assets/anymal_shank_r_1.obj",
    "anybotics_anymal_b/assets/anymal_shank_r_2.obj",
    "anybotics_anymal_b/assets/anymal_shank_r_3.obj",
    "anybotics_anymal_b/assets/anymal_thigh_l_0.obj",
    "anybotics_anymal_b/assets/anymal_thigh_l_1.obj",
    "anybotics_anymal_b/assets/anymal_thigh_l_2.obj",
    "anybotics_anymal_b/assets/anymal_thigh_l_3.obj",
    "anybotics_anymal_b/assets/anymal_thigh_l_4.obj",
    "anybotics_anymal_b/assets/anymal_thigh_l_5.obj",
    "anybotics_anymal_b/assets/anymal_thigh_r_0.obj",
    "anybotics_anymal_b/assets/anymal_thigh_r_1.obj",
    "anybotics_anymal_b/assets/anymal_thigh_r_2.obj",
    "anybotics_anymal_b/assets/anymal_thigh_r_3.obj",
    "anybotics_anymal_b/assets/anymal_thigh_r_4.obj",
    "anybotics_anymal_b/assets/anymal_thigh_r_5.obj",
    "anybotics_anymal_b/assets/base_uv_texture.png",
    "anybotics_anymal_b/assets/carbon_uv_texture.png",
    "anybotics_anymal_c/anybotics_anymal_c.xml",
    "anybotics_anymal_c/assets/base.png",
    "anybotics_anymal_c/assets/base_0.obj",
    "anybotics_anymal_c/assets/base_1.obj",
    "anybotics_anymal_c/assets/base_2.obj",
    "anybotics_anymal_c/assets/base_3.obj",
    "anybotics_anymal_c/assets/base_4.obj",
    "anybotics_anymal_c/assets/base_5.obj",
    "anybotics_anymal_c/assets/battery.obj",
    "anybotics_anymal_c/assets/battery.png",
    "anybotics_anymal_c/assets/bottom_shell.obj",
    "anybotics_anymal_c/assets/bottom_shell.png",
    "anybotics_anymal_c/assets/depth_camera.obj",
    "anybotics_anymal_c/assets/depth_camera.png",
    "anybotics_anymal_c/assets/drive.obj",
    "anybotics_anymal_c/assets/drive.png",
    "anybotics_anymal_c/assets/face.obj",
    "anybotics_anymal_c/assets/face.png",
    "anybotics_anymal_c/assets/foot.obj",
    "anybotics_anymal_c/assets/foot.png",
    "anybotics_anymal_c/assets/handle.obj",
    "anybotics_anymal_c/assets/handle.png",
    "anybotics_anymal_c/assets/hatch.obj",
    "anybotics_anymal_c/assets/hatch.png",
    "anybotics_anymal_c/assets/hip_l.obj",
    "anybotics_anymal_c/assets/hip_l.png",
    "anybotics_anymal_c/assets/hip_r.obj",
    "anybotics_anymal_c/assets/hip_r.png",
    "anybotics_anymal_c/assets/lidar.obj",
    "anybotics_anymal_c/assets/lidar.png",
    "anybotics_anymal_c/assets/lidar_cage.obj",
    "anybotics_anymal_c/assets/lidar_cage.png",
    "anybotics_anymal_c/assets/remote.obj",
    "anybotics_anymal_c/assets/remote.png",
    "anybotics_anymal_c/assets/shank_l.obj",
    "anybotics_anymal_c/assets/shank_l.png",
    "anybotics_anymal_c/assets/shank_r.obj",
    "anybotics_anymal_c/assets/shank_r.png",
    "anybotics_anymal_c/assets/thigh.obj",
    "anybotics_anymal_c/assets/thigh.png",
    "anybotics_anymal_c/assets/top_shell.obj",
    "anybotics_anymal_c/assets/top_shell.png",
    "anybotics_anymal_c/assets/wide_angle_camera.obj",
    "anybotics_anymal_c/assets/wide_angle_camera.png",
    "google_barkour_v0/assets/30degree_aframe__1default.stl",
    "google_barkour_v0/assets/abduction.stl",
    "google_barkour_v0/assets/aframe_side__1default.stl",
    "google_barkour_v0/assets/aframe_side__2default.stl",
    "google_barkour_v0/assets/body.stl",
    "google_barkour_v0/assets/broadjump_texture.png",
    "google_barkour_v0/assets/cone1__1default.stl",
    "google_barkour_v0/assets/foot.stl",
    "google_barkour_v0/assets/handle.stl",
    "google_barkour_v0/assets/head.stl",
    "google_barkour_v0/assets/head_mount.stl",
    "google_barkour_v0/assets/lower_leg_1to1.stl",
    "google_barkour_v0/assets/powercable.stl",
    "google_barkour_v0/assets/upper_left_1.stl",
    "google_barkour_v0/assets/upper_left_2.stl",
    "google_barkour_v0/assets/upper_left_3.stl",
    "google_barkour_v0/assets/upper_right_1.stl",
    "google_barkour_v0/assets/upper_right_2.stl",
    "google_barkour_v0/assets/upper_right_3.stl",
    "google_barkour_v0/google_barkour_v0.xml",
    "google_barkour_vb/assets/abduction.stl",
    "google_barkour_vb/assets/camera_cover.stl",
    "google_barkour_vb/assets/foot.stl",
    "google_barkour_vb/assets/handle.stl",
    "google_barkour_vb/assets/intel_realsense_depth_camera_d435.stl",
    "google_barkour_vb/assets/lower_leg.stl",
    "google_barkour_vb/assets/neck.stl",
    "google_barkour_vb/assets/torso.stl",
    "google_barkour_vb/assets/upper_leg.stl",
    "google_barkour_vb/assets/upper_leg_left.stl",
    "google_barkour_vb/assets/upper_leg_right.stl",
    "google_barkour_vb/google_barkour_vb.xml",
    "hexapod/hexapod.xml",
    "hexapod/meshes/base.stl",
    "hexapod/meshes/leg_0_left.stl",
    "hexapod/meshes/leg_0_right.stl",
    "hexapod/meshes/leg_1.stl",
    "hexapod/meshes/leg_2_left.stl",
    "hexapod/meshes/leg_2_right.stl",
    "hexapod/meshes/motor.stl",
    "mab_silver_badger/mab_silver_badger.xml",
    "mab_silver_badger/meshes/body.stl",
    "mab_silver_badger/meshes/l0l.stl",
    "mab_silver_badger/meshes/l0r.stl",
    "mab_silver_badger/meshes/l1lf.stl",
    "mab_silver_badger/meshes/l1r.stl",
    "mab_silver_badger/meshes/l2l.stl",
    "mab_silver_badger/meshes/l2r.stl",
    "mab_silver_badger/meshes/rear.stl",
    "pal_robotics_talos/meshes/arm/arm_1.STL",
    "pal_robotics_talos/meshes/arm/arm_1_collision.STL",
    "pal_robotics_talos/meshes/arm/arm_2.STL",
    "pal_robotics_talos/meshes/arm/arm_2_collision.STL",
    "pal_robotics_talos/meshes/arm/arm_3.STL",
    "pal_robotics_talos/meshes/arm/arm_3_collision.STL",
    "pal_robotics_talos/meshes/arm/arm_4.STL",
    "pal_robotics_talos/meshes/arm/arm_4_collision.STL",
    "pal_robotics_talos/meshes/arm/arm_5.STL",
    "pal_robotics_talos/meshes/arm/arm_5_collision.STL",
    "pal_robotics_talos/meshes/arm/arm_6.STL",
    "pal_robotics_talos/meshes/arm/arm_6_collision.STL",
    "pal_robotics_talos/meshes/arm/arm_7.STL",
    "pal_robotics_talos/meshes/arm/arm_7_collision.STL",
    "pal_robotics_talos/meshes/bart.STL",
    "pal_robotics_talos/meshes/bart_light.STL",
    "pal_robotics_talos/meshes/dummy_hand.STL",
    "pal_robotics_talos/meshes/femur.STL",
    "pal_robotics_talos/meshes/femur_light.STL",
    "pal_robotics_talos/meshes/foot.STL",
    "pal_robotics_talos/meshes/gripper/base_link_gripper.STL",
    "pal_robotics_talos/meshes/gripper/base_link_gripper_collision.STL",
    "pal_robotics_talos/meshes/gripper/fingertip.STL",
    "pal_robotics_talos/meshes/gripper/fingertip_collision.STL",
    "pal_robotics_talos/meshes/gripper/gripper_motor_double.STL",
    "pal_robotics_talos/meshes/gripper/gripper_motor_double_collision.STL",
    "pal_robotics_talos/meshes/gripper/gripper_motor_single.STL",
    "pal_robotics_talos/meshes/gripper/gripper_motor_single_collision.STL",
    "pal_robotics_talos/meshes/gripper/gripper_simple.STL",
    "pal_robotics_talos/meshes/gripper/gripper_simple_collision.STL",
    "pal_robotics_talos/meshes/gripper/gripper_tweezers.STL",
    "pal_robotics_talos/meshes/gripper/gripper_tweezers_collision.STL",
    "pal_robotics_talos/meshes/gripper/inner_double.STL",
    "pal_robotics_talos/meshes/gripper/inner_double_collision.STL",
    "pal_robotics_talos/meshes/gripper/inner_single.STL",
    "pal_robotics_talos/meshes/gripper/inner_single_collision.STL",
    "pal_robotics_talos/meshes/head/head_1.stl",
    "pal_robotics_talos/meshes/head/head_1_collision.stl",
    "pal_robotics_talos/meshes/head/head_2.stl",
    "pal_robotics_talos/meshes/head/head_2_collision.stl",
    "pal_robotics_talos/meshes/head/head_2_lidar.stl",
    "pal_robotics_talos/meshes/head/head_2_lidar_collision.stl",
    "pal_robotics_talos/meshes/hipZ.STL",
    "pal_robotics_talos/meshes/hipZ_light.STL",
    "pal_robotics_talos/meshes/sensors/orbbec/orbbec.STL",
    "pal_robotics_talos/meshes/sensors/xtion_pro_live/xtion_pro_live.png",
    "pal_robotics_talos/meshes/super-simple-bart.STL",
    "pal_robotics_talos/meshes/super-simple-femur.STL",
    "pal_robotics_talos/meshes/super-simple-tibia.STL",
    "pal_robotics_talos/meshes/talos_dummy.stl",
    "pal_robotics_talos/meshes/tibia.STL",
    "pal_robotics_talos/meshes/tibia_light.STL",
    "pal_robotics_talos/meshes/torso/base_link.STL",
    "pal_robotics_talos/meshes/torso/base_link_collision.STL",
    "pal_robotics_talos/meshes/torso/torso_1.STL",
    "pal_robotics_talos/meshes/torso/torso_2.STL",
    "pal_robotics_talos/meshes/torso/torso_2_collision.STL",
    "pal_robotics_talos/meshes/v2/ankle_X_collision.stl",
    "pal_robotics_talos/meshes/v2/ankle_X_lo_res.stl",
    "pal_robotics_talos/meshes/v2/ankle_Y_collision.stl",
    "pal_robotics_talos/meshes/v2/ankle_Y_lo_res.stl",
    "pal_robotics_talos/meshes/v2/base_link_collision.stl",
    "pal_robotics_talos/meshes/v2/base_link_lo_res.stl",
    "pal_robotics_talos/meshes/v2/hip_x_collision.stl",
    "pal_robotics_talos/meshes/v2/hip_x_lo_res.stl",
    "pal_robotics_talos/meshes/v2/hip_y_collision.stl",
    "pal_robotics_talos/meshes/v2/hip_y_lo_res.stl",
    "pal_robotics_talos/meshes/v2/hip_z_collision.stl",
    "pal_robotics_talos/meshes/v2/hip_z_lo_res.stl",
    "pal_robotics_talos/meshes/v2/knee_collision.stl",
    "pal_robotics_talos/meshes/v2/knee_lo_res.stl",
    "pal_robotics_talos/pal_robotics_talos.xml",
    "petoi_bittle/assets/base_frame.001.obj",
    "petoi_bittle/assets/base_frame.obj",
    "petoi_bittle/assets/battery.001.obj",
    "petoi_bittle/assets/battery.obj",
    "petoi_bittle/assets/cover.001.obj",
    "petoi_bittle/assets/cover.obj",
    "petoi_bittle/assets/left_knee.001.obj",
    "petoi_bittle/assets/left_knee.002.obj",
    "petoi_bittle/assets/left_knee.003.obj",
    "petoi_bittle/assets/left_knee.obj",
    "petoi_bittle/assets/right_knee.001.obj",
    "petoi_bittle/assets/right_knee.002.obj",
    "petoi_bittle/assets/right_knee.003.obj",
    "petoi_bittle/assets/right_knee.obj",
    "petoi_bittle/assets/shoulder.001.obj",
    "petoi_bittle/assets/shoulder.002.obj",
    "petoi_bittle/assets/shoulder.003.obj",
    "petoi_bittle/assets/shoulder.004.obj",
    "petoi_bittle/assets/shoulder.005.obj",
    "petoi_bittle/assets/shoulder.006.obj",
    "petoi_bittle/assets/shoulder.007.obj",
    "petoi_bittle/assets/shoulder.obj",
    "petoi_bittle/petoi_bittle.xml",
    "robotis_op3/assets/body.stl",
    "robotis_op3/assets/h1.stl",
    "robotis_op3/assets/h2.stl",
    "robotis_op3/assets/la1.stl",
    "robotis_op3/assets/la2.stl",
    "robotis_op3/assets/la3.stl",
    "robotis_op3/assets/ll1.stl",
    "robotis_op3/assets/ll2.stl",
    "robotis_op3/assets/ll3.stl",
    "robotis_op3/assets/ll4.stl",
    "robotis_op3/assets/ll5.stl",
    "robotis_op3/assets/ll6.stl",
    "robotis_op3/assets/ra1.stl",
    "robotis_op3/assets/ra2.stl",
    "robotis_op3/assets/ra3.stl",
    "robotis_op3/assets/rl1.stl",
    "robotis_op3/assets/rl2.stl",
    "robotis_op3/assets/rl3.stl",
    "robotis_op3/assets/rl4.stl",
    "robotis_op3/assets/rl5.stl",
    "robotis_op3/assets/rl6.stl",
    "robotis_op3/assets/simplified_convex/body.stl",
    "robotis_op3/assets/simplified_convex/body_sub1.stl",
    "robotis_op3/assets/simplified_convex/body_sub2.stl",
    "robotis_op3/assets/simplified_convex/body_sub3.stl",
    "robotis_op3/assets/simplified_convex/body_sub4.stl",
    "robotis_op3/assets/simplified_convex/h1.stl",
    "robotis_op3/assets/simplified_convex/h2.stl",
    "robotis_op3/assets/simplified_convex/h2_sub1.stl",
    "robotis_op3/assets/simplified_convex/h2_sub2.stl",
    "robotis_op3/assets/simplified_convex/la1.stl",
    "robotis_op3/assets/simplified_convex/la2.stl",
    "robotis_op3/assets/simplified_convex/la3.stl",
    "robotis_op3/assets/simplified_convex/ll1.stl",
    "robotis_op3/assets/simplified_convex/ll2.stl",
    "robotis_op3/assets/simplified_convex/ll3.stl",
    "robotis_op3/assets/simplified_convex/ll4.stl",
    "robotis_op3/assets/simplified_convex/ll5.stl",
    "robotis_op3/assets/simplified_convex/ll6.stl",
    "robotis_op3/assets/simplified_convex/ra1.stl",
    "robotis_op3/assets/simplified_convex/ra2.stl",
    "robotis_op3/assets/simplified_convex/ra3.stl",
    "robotis_op3/assets/simplified_convex/rl1.stl",
    "robotis_op3/assets/simplified_convex/rl2.stl",
    "robotis_op3/assets/simplified_convex/rl3.stl",
    "robotis_op3/assets/simplified_convex/rl4.stl",
    "robotis_op3/assets/simplified_convex/rl5.stl",
    "robotis_op3/assets/simplified_convex/rl6.stl",
    "robotis_op3/robotis_op3.xml",
    "softbank_nao_v5/meshes/HeadPitch_0.10.stl",
    "softbank_nao_v5/meshes/HeadYaw_0.10.stl",
    "softbank_nao_v5/meshes/LAnklePitch_0.10.stl",
    "softbank_nao_v5/meshes/LAnkleRoll_0.10.stl",
    "softbank_nao_v5/meshes/LElbowRoll_0.10.stl",
    "softbank_nao_v5/meshes/LFinger11_0.10.stl",
    "softbank_nao_v5/meshes/LFinger12_0.10.stl",
    "softbank_nao_v5/meshes/LFinger13_0.10.stl",
    "softbank_nao_v5/meshes/LFinger21_0.10.stl",
    "softbank_nao_v5/meshes/LFinger22_0.10.stl",
    "softbank_nao_v5/meshes/LFinger23_0.10.stl",
    "softbank_nao_v5/meshes/LHipPitch_0.10.stl",
    "softbank_nao_v5/meshes/LHipRoll_0.10.stl",
    "softbank_nao_v5/meshes/LHipYawPitch_0.10.stl",
    "softbank_nao_v5/meshes/LKneePitch_0.10.stl",
    "softbank_nao_v5/meshes/LShoulderPitch_0.10.stl",
    "softbank_nao_v5/meshes/LShoulderRoll_0.10.stl",
    "softbank_nao_v5/meshes/LThumb1_0.10.stl",
    "softbank_nao_v5/meshes/LThumb2_0.10.stl",
    "softbank_nao_v5/meshes/LWristYaw_0.10.stl",
    "softbank_nao_v5/meshes/RAnklePitch_0.10.stl",
    "softbank_nao_v5/meshes/RAnkleRoll_0.10.stl",
    "softbank_nao_v5/meshes/RElbowRoll_0.10.stl",
    "softbank_nao_v5/meshes/RFinger11_0.10.stl",
    "softbank_nao_v5/meshes/RFinger12_0.10.stl",
    "softbank_nao_v5/meshes/RFinger13_0.10.stl",
    "softbank_nao_v5/meshes/RFinger21_0.10.stl",
    "softbank_nao_v5/meshes/RFinger22_0.10.stl",
    "softbank_nao_v5/meshes/RFinger23_0.10.stl",
    "softbank_nao_v5/meshes/RHipPitch_0.10.stl",
    "softbank_nao_v5/meshes/RHipRoll_0.10.stl",
    "softbank_nao_v5/meshes/RHipYawPitch_0.10.stl",
    "softbank_nao_v5/meshes/RKneePitch_0.10.stl",
    "softbank_nao_v5/meshes/RShoulderPitch_0.10.stl",
    "softbank_nao_v5/meshes/RShoulderRoll_0.10.stl",
    "softbank_nao_v5/meshes/RThumb1_0.10.stl",
    "softbank_nao_v5/meshes/RThumb2_0.10.stl",
    "softbank_nao_v5/meshes/RWristYaw_0.10.stl",
    "softbank_nao_v5/meshes/Torso_0.10.stl",
    "softbank_nao_v5/softbank_nao_v5.xml",
    "unitree_a1/assets/calf.obj",
    "unitree_a1/assets/hip.obj",
    "unitree_a1/assets/thigh.obj",
    "unitree_a1/assets/thigh_mirror.obj",
    "unitree_a1/assets/trunk.obj",
    "unitree_a1/assets/trunk_A1.png",
    "unitree_a1/meshes/calf.stl",
    "unitree_a1/meshes/hip.stl",
    "unitree_a1/meshes/thigh.stl",
    "unitree_a1/meshes/thigh_mirror.stl",
    "unitree_a1/meshes/trunk.stl",
    "unitree_a1/unitree_a1.xml",
    "unitree_g1/meshes/head_link.STL",
    "unitree_g1/meshes/left_ankle_pitch_link.STL",
    "unitree_g1/meshes/left_ankle_roll_link.STL",
    "unitree_g1/meshes/left_elbow_pitch_link.STL",
    "unitree_g1/meshes/left_elbow_roll_link.STL",
    "unitree_g1/meshes/left_five_link.STL",
    "unitree_g1/meshes/left_four_link.STL",
    "unitree_g1/meshes/left_hip_pitch_link.STL",
    "unitree_g1/meshes/left_hip_roll_link.STL",
    "unitree_g1/meshes/left_hip_yaw_link.STL",
    "unitree_g1/meshes/left_knee_link.STL",
    "unitree_g1/meshes/left_one_link.STL",
    "unitree_g1/meshes/left_palm_link.STL",
    "unitree_g1/meshes/left_shoulder_pitch_link.STL",
    "unitree_g1/meshes/left_shoulder_roll_link.STL",
    "unitree_g1/meshes/left_shoulder_yaw_link.STL",
    "unitree_g1/meshes/left_six_link.STL",
    "unitree_g1/meshes/left_three_link.STL",
    "unitree_g1/meshes/left_two_link.STL",
    "unitree_g1/meshes/left_zero_link.STL",
    "unitree_g1/meshes/logo_link.STL",
    "unitree_g1/meshes/pelvis.STL",
    "unitree_g1/meshes/pelvis_contour_link.STL",
    "unitree_g1/meshes/right_ankle_pitch_link.STL",
    "unitree_g1/meshes/right_ankle_roll_link.STL",
    "unitree_g1/meshes/right_elbow_pitch_link.STL",
    "unitree_g1/meshes/right_elbow_roll_link.STL",
    "unitree_g1/meshes/right_five_link.STL",
    "unitree_g1/meshes/right_four_link.STL",
    "unitree_g1/meshes/right_hip_pitch_link.STL",
    "unitree_g1/meshes/right_hip_roll_link.STL",
    "unitree_g1/meshes/right_hip_yaw_link.STL",
    "unitree_g1/meshes/right_knee_link.STL",
    "unitree_g1/meshes/right_one_link.STL",
    "unitree_g1/meshes/right_palm_link.STL",
    "unitree_g1/meshes/right_shoulder_pitch_link.STL",
    "unitree_g1/meshes/right_shoulder_roll_link.STL",
    "unitree_g1/meshes/right_shoulder_yaw_link.STL",
    "unitree_g1/meshes/right_six_link.STL",
    "unitree_g1/meshes/right_three_link.STL",
    "unitree_g1/meshes/right_two_link.STL",
    "unitree_g1/meshes/right_zero_link.STL",
    "unitree_g1/meshes/torso_link.STL",
    "unitree_g1/unitree_g1.xml",
    "unitree_go1/assets/calf.stl",
    "unitree_go1/assets/hip.stl",
    "unitree_go1/assets/thigh.stl",
    "unitree_go1/assets/thigh_mirror.stl",
    "unitree_go1/assets/trunk.stl",
    "unitree_go1/unitree_go1.xml",
    "unitree_go2/assets/base_0.obj",
    "unitree_go2/assets/base_1.obj",
    "unitree_go2/assets/base_2.obj",
    "unitree_go2/assets/base_3.obj",
    "unitree_go2/assets/base_4.obj",
    "unitree_go2/assets/calf_0.obj",
    "unitree_go2/assets/calf_1.obj",
    "unitree_go2/assets/calf_mirror_0.obj",
    "unitree_go2/assets/calf_mirror_1.obj",
    "unitree_go2/assets/foot.obj",
    "unitree_go2/assets/hip_0.obj",
    "unitree_go2/assets/hip_1.obj",
    "unitree_go2/assets/thigh_0.obj",
    "unitree_go2/assets/thigh_1.obj",
    "unitree_go2/assets/thigh_mirror_0.obj",
    "unitree_go2/assets/thigh_mirror_1.obj",
    "unitree_go2/unitree_go2.xml",
    "unitree_h1/assets/left_ankle_link.stl",
    "unitree_h1/assets/left_elbow_link.stl",
    "unitree_h1/assets/left_hip_pitch_link.stl",
    "unitree_h1/assets/left_hip_roll_link.stl",
    "unitree_h1/assets/left_hip_yaw_link.stl",
    "unitree_h1/assets/left_knee_link.stl",
    "unitree_h1/assets/left_shoulder_pitch_link.stl",
    "unitree_h1/assets/left_shoulder_roll_link.stl",
    "unitree_h1/assets/left_shoulder_yaw_link.stl",
    "unitree_h1/assets/logo_link.stl",
    "unitree_h1/assets/pelvis.stl",
    "unitree_h1/assets/right_ankle_link.stl",
    "unitree_h1/assets/right_elbow_link.stl",
    "unitree_h1/assets/right_hip_pitch_link.stl",
    "unitree_h1/assets/right_hip_roll_link.stl",
    "unitree_h1/assets/right_hip_yaw_link.stl",
    "unitree_h1/assets/right_knee_link.stl",
    "unitree_h1/assets/right_shoulder_pitch_link.stl",
    "unitree_h1/assets/right_shoulder_roll_link.stl",
    "unitree_h1/assets/right_shoulder_yaw_link.stl",
    "unitree_h1/assets/torso_link.stl",
    "unitree_h1/unitree_h1.xml"
  ];

  let requests = allFiles.map((url) => fetch("robots/" + url));
  let responses = await Promise.all(requests);
  for (let i = 0; i < responses.length; i++) {
      let split = allFiles[i].split("/");
      let working = '/working/';
      for (let f = 0; f < split.length - 1; f++) {
          working += split[f];
          if (!mujoco.FS.analyzePath(working).exists) { mujoco.FS.mkdir(working); }
          working += "/";
      }

      if (allFiles[i].endsWith(".png") || allFiles[i].endsWith(".stl") || allFiles[i].endsWith(".STL") || allFiles[i].endsWith(".skn") || allFiles[i].endsWith(".obj")) {
          mujoco.FS.writeFile("/working/" + allFiles[i], new Uint8Array(await responses[i].arrayBuffer()));
      } else {
          mujoco.FS.writeFile("/working/" + allFiles[i], await responses[i].text());
      }
  }
}

export function getPosition(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 2],
      -buffer[(index * 3) + 1]);
  } else {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 1],
       buffer[(index * 3) + 2]);
  }
}

export function getQuaternion(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      -buffer[(index * 4) + 1],
      -buffer[(index * 4) + 3],
       buffer[(index * 4) + 2],
      -buffer[(index * 4) + 0]);
  } else {
    return target.set(
       buffer[(index * 4) + 0],
       buffer[(index * 4) + 1],
       buffer[(index * 4) + 2],
       buffer[(index * 4) + 3]);
  }
}