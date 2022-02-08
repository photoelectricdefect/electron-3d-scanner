const { ipcRenderer } = require('electron');
import * as THREE from '../../node_modules/three/build/three.module.js';
import { OrbitControls } from './external/OrbitControls.js';
const math = require('mathjs');

let scan = () => {
  const onWindowResize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    // camera.updateProjectionMatrix();
    renderer.setSize( window.innerWidth, window.innerHeight );
  }
  
  const animate=()=> {
    requestAnimationFrame( animate );
    // controls.update(); // only required if controls.enableDamping = true, or if controls.autoRotate = true
    renderer.render( scene, camera );
    
    if(resetControls) {
      resetControls=false;
      controls.reset();
    }
  }

  const dot =(a,b)=> {
    let result=0;

    for(let i=0; i<a.length; i++) {
      result+=a[i]*b[i];
    }

    return result;
  };
  
  const diference=(a,b)=>{
    let result=[];

    for(let i=0; i<a.length; i++) {
      result.push(a[i]-b[i]);
    }

    return result;
  };

  let controls;
  let container;
  let camera, scene, renderer;
  let points;
  let rotationAxisOrigin;
  let rotationAxisDirection;
  let rotationAxisRadius;
  let cameraOrigin;
  let objectAnchor;
  let polarGridHelper;
  let resetControls=false;

	return {
      init: ()=> {
        container = document.getElementById( 'container' );
        camera = new THREE.PerspectiveCamera( 27, window.innerWidth / window.innerHeight, 5, 3500 );
        camera.position.set( 0, 0, 0 );
        
        scene = new THREE.Scene();
        scene.background = new THREE.Color( "#7b8184" );
        // scene.fog = new THREE.Fog( 0xcccccc, 2000, 3500 );
        
        renderer = new THREE.WebGLRenderer();
        renderer.setPixelRatio( window.devicePixelRatio );
        renderer.setSize( window.innerWidth, window.innerHeight );
        
        container.appendChild( renderer.domElement );
        
        controls = new OrbitControls( camera, renderer.domElement );
        // controls.listenToKeyEvents( window ); // optional
        // controls.enableDamping = true; // an animation loop is required when either damping or auto-rotation are enabled
        // controls.dampingFactor = 0.05;
        controls.screenSpacePanning = false;
        controls.update();
        // controls.minDistance = 100;
        controls.maxDistance = 1000;
        // controls.maxPolarAngle = Math.PI / 2;			

        const MAX_POINTS=1000000;
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array( MAX_POINTS * 3 );
        const colors = new Float32Array( MAX_POINTS * 3 );
        geometry.setAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
        geometry.setAttribute( 'color', new THREE.BufferAttribute( colors, 3 ,true) );
        const material = new THREE.PointsMaterial( { size: 6, vertexColors: true } );
        points = new THREE.Points( geometry, material );
        scene.add( points );
        const color = new THREE.Color();
        
        let npoints=0;
        let arrayFrom=0;
        let arrayTo=0;
  
        window.addEventListener( 'resize', onWindowResize );

        ipcRenderer.on('data', (e,d)=> {
          console.log(d);
          npoints+=d.points.positions[0].length;
          arrayFrom=arrayTo;
          arrayTo+=d.points.positions[0].length*3;
          const positionsArray=points.geometry.attributes.position.array;
          const colorsArray=points.geometry.attributes.color.array;
        
          for(let i=arrayFrom,j=arrayFrom,index=0;i<arrayTo;index++) {
            positionsArray[i++]=d.points.positions[0][index];
            positionsArray[i++]=d.points.positions[1][index];
            positionsArray[i++]=d.points.positions[2][index];
            let b=d.points.colors[0][index],g=d.points.colors[1][index],r=d.points.colors[2][index];
            colorsArray[j++]=r/255.;
            colorsArray[j++]=g/255.;
            colorsArray[j++]=b/255.;
          }

          points.geometry.setDrawRange( 0, npoints );
          points.geometry.attributes.position.needsUpdate = true;
          points.geometry.attributes.color.needsUpdate = true;
          points.geometry.computeBoundingSphere();            
          points.geometry.computeBoundingBox();
          controls.saveState();
          resetControls=true;
        });

        ipcRenderer.on('init', (e,d)=> {
          rotationAxisOrigin=d.calibrationData.rotation_axis_origin;
          rotationAxisDirection=d.calibrationData.rotation_axis_direction;
          objectAnchor=d.calibrationData.object_anchor;
          cameraOrigin=d.calibrationData.camera_origin;
          rotationAxisRadius=d.calibrationData.rotation_axis_radius;

          // let cameraOriginVector=new THREE.Vector3(cameraOrigin[0], cameraOrigin[1], cameraOrigin[2])
          // let objectAnchorVector=new THREE.Vector3(objectAnchor[0], objectAnchor[1], objectAnchor[2])          
          
          // let objectCameraDirectionVector=new THREE.Vector3();
          // objectCameraDirectionVector.subVectors( cameraOriginVector, objectAnchorVector ).normalize();



          polarGridHelper = new THREE.PolarGridHelper( rotationAxisRadius, 16, 8, 64, 0xFFFFFF, 0xFFFFFF );
          polarGridHelper.position.z = objectAnchor[2];
          polarGridHelper.position.x = objectAnchor[0];
          scene.add( polarGridHelper );

          // let v=objectAnchorVector.clone().add(objectCameraDirectionVector.multiplyScalar(-10));
          // console.log(v);
          // console.log(cameraOrigin);

          camera.position.set( cameraOrigin[0], cameraOrigin[1], cameraOrigin[2] );
          // camera.zoom=0.5;
          controls.target = new THREE.Vector3(objectAnchor[0], objectAnchor[1], objectAnchor[2]);
          controls.update();
        });
        
        animate();               
		  }
    };
};


export {scan}; 