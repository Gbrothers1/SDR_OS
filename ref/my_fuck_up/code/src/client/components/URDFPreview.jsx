import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';

/**
 * URDFPreview - Renders a 3D preview of a robot from its URDF file
 * Used in SessionCard for training session previews
 */
export default function URDFPreview({
  urdfPath,
  width = '100%',
  height = '100%',
  cameraDistance = 1.5,
  cameraHeight = 0.8,
  autoRotate = true,
  backgroundColor = '#1a1a1a',
}) {
  const containerRef = useRef(null);
  const sceneRef = useRef(null);
  const rendererRef = useRef(null);
  const cameraRef = useRef(null);
  const robotRef = useRef(null);
  const animationRef = useRef(null);

  useEffect(() => {
    if (!containerRef.current || !urdfPath) return;

    const container = containerRef.current;
    const width = container.clientWidth;
    const height = container.clientHeight;

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(backgroundColor);
    sceneRef.current = scene;

    // Camera
    const camera = new THREE.PerspectiveCamera(50, width / height, 0.01, 100);
    camera.position.set(cameraDistance, cameraHeight, cameraDistance);
    camera.lookAt(0, cameraHeight * 0.5, 0);
    cameraRef.current = camera;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 5);
    directionalLight.castShadow = true;
    scene.add(directionalLight);

    const fillLight = new THREE.DirectionalLight(0xffffff, 0.3);
    fillLight.position.set(-5, 5, -5);
    scene.add(fillLight);

    // Ground plane (subtle)
    const groundGeometry = new THREE.PlaneGeometry(10, 10);
    const groundMaterial = new THREE.MeshStandardMaterial({
      color: 0x222222,
      transparent: true,
      opacity: 0.5,
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    scene.add(ground);

    // Load URDF
    const loader = new URDFLoader();
    loader.loadMeshCb = (path, manager, done) => {
      // Handle mesh loading - try to find the mesh file
      const meshLoader = new THREE.FileLoader(manager);
      meshLoader.setResponseType('arraybuffer');

      // The path comes from the URDF, we need to resolve it relative to assets
      let resolvedPath = path;
      if (!path.startsWith('/') && !path.startsWith('http')) {
        // Relative path - resolve from the URDF directory
        const urdfDir = urdfPath.substring(0, urdfPath.lastIndexOf('/'));
        resolvedPath = `/${urdfDir}/${path}`;
      }

      // For STL files
      if (path.toLowerCase().endsWith('.stl')) {
        import('three/examples/jsm/loaders/STLLoader').then(({ STLLoader }) => {
          const stlLoader = new STLLoader(manager);
          stlLoader.load(
            resolvedPath,
            (geometry) => {
              const material = new THREE.MeshStandardMaterial({
                color: 0x888888,
                metalness: 0.3,
                roughness: 0.7,
              });
              const mesh = new THREE.Mesh(geometry, material);
              mesh.castShadow = true;
              mesh.receiveShadow = true;
              done(mesh);
            },
            undefined,
            (err) => {
              console.warn('Failed to load STL:', resolvedPath, err);
              done(null);
            }
          );
        });
      }
      // For DAE (Collada) files
      else if (path.toLowerCase().endsWith('.dae')) {
        import('three/examples/jsm/loaders/ColladaLoader').then(({ ColladaLoader }) => {
          const daeLoader = new ColladaLoader(manager);
          daeLoader.load(
            resolvedPath,
            (collada) => {
              const model = collada.scene;
              model.traverse((child) => {
                if (child.isMesh) {
                  child.castShadow = true;
                  child.receiveShadow = true;
                }
              });
              done(model);
            },
            undefined,
            (err) => {
              console.warn('Failed to load DAE:', resolvedPath, err);
              done(null);
            }
          );
        });
      }
      // For OBJ files
      else if (path.toLowerCase().endsWith('.obj')) {
        import('three/examples/jsm/loaders/OBJLoader').then(({ OBJLoader }) => {
          const objLoader = new OBJLoader(manager);
          objLoader.load(
            resolvedPath,
            (obj) => {
              obj.traverse((child) => {
                if (child.isMesh) {
                  child.material = new THREE.MeshStandardMaterial({
                    color: 0x888888,
                    metalness: 0.3,
                    roughness: 0.7,
                  });
                  child.castShadow = true;
                  child.receiveShadow = true;
                }
              });
              done(obj);
            },
            undefined,
            (err) => {
              console.warn('Failed to load OBJ:', resolvedPath, err);
              done(null);
            }
          );
        });
      }
      // Fallback - create a simple box
      else {
        console.warn('Unsupported mesh format:', path);
        const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
        const material = new THREE.MeshStandardMaterial({ color: 0x888888 });
        done(new THREE.Mesh(geometry, material));
      }
    };

    // Load the URDF
    loader.load(`/${urdfPath}`, (robot) => {
      robotRef.current = robot;

      // Center the robot
      const box = new THREE.Box3().setFromObject(robot);
      const center = box.getCenter(new THREE.Vector3());
      robot.position.sub(center);
      robot.position.y = -box.min.y; // Place on ground

      // Apply default materials if missing
      robot.traverse((child) => {
        if (child.isMesh) {
          if (!child.material || child.material.type === 'MeshBasicMaterial') {
            child.material = new THREE.MeshStandardMaterial({
              color: 0x666666,
              metalness: 0.4,
              roughness: 0.6,
            });
          }
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });

      scene.add(robot);
    }, undefined, (err) => {
      console.error('Failed to load URDF:', urdfPath, err);
    });

    // Animation loop
    let angle = 0;
    const animate = () => {
      animationRef.current = requestAnimationFrame(animate);

      if (autoRotate && robotRef.current) {
        angle += 0.005;
        camera.position.x = Math.sin(angle) * cameraDistance;
        camera.position.z = Math.cos(angle) * cameraDistance;
        camera.lookAt(0, cameraHeight * 0.5, 0);
      }

      renderer.render(scene, camera);
    };
    animate();

    // Handle resize
    const handleResize = () => {
      if (!container) return;
      const newWidth = container.clientWidth;
      const newHeight = container.clientHeight;
      camera.aspect = newWidth / newHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(newWidth, newHeight);
    };

    const resizeObserver = new ResizeObserver(handleResize);
    resizeObserver.observe(container);

    // Cleanup
    return () => {
      resizeObserver.disconnect();
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
      if (rendererRef.current) {
        rendererRef.current.dispose();
        container.removeChild(rendererRef.current.domElement);
      }
      if (sceneRef.current) {
        sceneRef.current.traverse((obj) => {
          if (obj.geometry) obj.geometry.dispose();
          if (obj.material) {
            if (Array.isArray(obj.material)) {
              obj.material.forEach(m => m.dispose());
            } else {
              obj.material.dispose();
            }
          }
        });
      }
    };
  }, [urdfPath, cameraDistance, cameraHeight, autoRotate, backgroundColor]);

  return (
    <div
      ref={containerRef}
      style={{
        width,
        height,
        overflow: 'hidden',
        borderRadius: 'inherit',
      }}
    />
  );
}
