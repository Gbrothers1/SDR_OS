#!/usr/bin/env python3
"""
Genesis-ROS Bridge for SDR_OS

Integrates genesis_ros simulation with SDR_OS web interface, providing:
- ROS2 topic subscriptions for joint states and sensors
- ROS2 topic publishing for joint commands
- WebSocket streaming for camera frames
- Socket.io integration for gamepad control
- HIL action blending with existing SDR_OS pipeline
"""

import asyncio
import json
import time
import logging
from pathlib import Path
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
import numpy as np
import cv2
import yaml

# ROS2 imports (wrapped in try/except for systems without ROS2)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState, Image, Imu
    from geometry_msgs.msg import WrenchStamped, PoseStamped
    from std_srvs.srv import Empty, SetBool, Trigger
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 not available - running in mock mode")

# WebSocket and Socket.io
try:
    import websockets
    from websockets.asyncio.server import serve as ws_serve
except ImportError:
    websockets = None
    ws_serve = None
    logging.warning("websockets not installed")

try:
    import socketio
except ImportError:
    socketio = None
    logging.warning("python-socketio not installed")

# Import SDR_OS action pipeline
import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "rl"))
from actions import ActionSpec, ActionRouter, TeleopBridge
from actions.action_spec import AnomalyDetector, AnomalyConfig

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class BridgeConfig:
    """Configuration for the Genesis-ROS bridge."""
    # WebSocket settings
    ws_host: str = "0.0.0.0"
    ws_port: int = 9091
    
    # Socket.io settings
    sdr_os_url: str = "http://localhost:3000"
    
    # ROS2 settings
    robot_config_path: Optional[str] = None
    control_mode: str = "joint_velocity"  # joint_velocity, cartesian_velocity
    
    # Rendering settings
    jpeg_quality: int = 85
    target_fps: int = 30
    
    # RL settings
    enable_rl: bool = True
    policy_checkpoint: Optional[str] = None
    
    # Data logging
    enable_logging: bool = False
    log_dir: str = "rl/datasets"


class GenesisROSBridge:
    """
    Bridge between genesis_ros simulation and SDR_OS web interface.
    
    Architecture:
    - Subscribes to ROS2 topics from genesis_ros (joint states, sensors, cameras)
    - Publishes ROS2 joint commands with HIL blending
    - Streams camera frames via WebSocket to web UI
    - Receives gamepad input via Socket.io
    - Integrates ActionRouter for teleop/policy blending
    """
    
    def __init__(self, config: Optional[BridgeConfig] = None):
        self.config = config or BridgeConfig()
        
        # Check ROS2 availability
        if not ROS2_AVAILABLE:
            raise RuntimeError(
                "ROS2 not available. Please install ROS2 and genesis_ros. "
                "See docs/GENESIS_ROS_SETUP.md"
            )
        
        # Initialize ROS2 node
        self.node = None
        self.cv_bridge = None
        
        # Robot configuration
        self.robot_config = None
        self.num_joints = 0
        self.joint_names = []
        
        # ROS2 publishers
        self.joint_cmd_pub = None
        
        # ROS2 subscribers
        self.joint_state_sub = None
        self.camera_subs = {}
        self.imu_sub = None
        self.ft_sub = None
        
        # ROS2 service clients
        self.reset_client = None
        self.pause_client = None
        self.ik_client = None
        
        # State
        self.current_joint_state = None
        self.current_obs = None
        self.current_cameras = {}
        self.current_imu = None
        self.current_ft = None
        
        # Action pipeline (SDR_OS)
        self.action_spec = ActionSpec()
        self.action_router = ActionRouter()
        self.teleop_bridge = TeleopBridge()
        self.anomaly_detector = AnomalyDetector(AnomalyConfig())
        
        # Policy
        self.policy = None
        self.policy_device = None
        self.gpu_pipeline: Optional['GpuFramePipeline'] = None

        # WebSocket clients
        self.ws_clients = set()
        
        # Socket.io client
        self.sio = None
        
        # Timing
        self.last_frame_time = 0
        self.frame_interval = 1.0 / self.config.target_fps
        
        # Statistics
        self.step_count = 0
        self.fps = 0
        self.fps_counter = 0
        self.fps_time = time.time()
        
        # Load robot config
        if self.config.robot_config_path:
            self.load_robot_config(self.config.robot_config_path)

    def set_gpu_pipeline(self, pipeline: 'GpuFramePipeline'):
        """Set GPU pipeline for accelerated encoding (called when Genesis is active)."""
        self.gpu_pipeline = pipeline
        logger.info(f"GPU pipeline set: nvjpeg={pipeline.has_nvjpeg}")

    def load_robot_config(self, config_path: str):
        """Load robot configuration from YAML file."""
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Robot config not found: {config_path}")
        
        with open(config_file) as f:
            self.robot_config = yaml.safe_load(f)
        
        # Extract joint info
        robot_info = self.robot_config.get('robot', {})
        self.joint_names = robot_info.get('joints', [])
        self.num_joints = len(self.joint_names)
        
        logger.info(f"Loaded robot config: {robot_info.get('name')}")
        logger.info(f"Joints: {self.num_joints} - {self.joint_names}")
    
    def initialize_ros2(self):
        """Initialize ROS2 node and communications."""
        if self.node is None:
            rclpy.init()
            self.node = rclpy.create_node('sdr_genesis_bridge')
            self.cv_bridge = CvBridge()
            logger.info("ROS2 node initialized")
        
        # Create QoS profile for real-time communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.joint_cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )
        logger.info("Created joint command publisher: /joint_commands")
        
        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )
        logger.info("Subscribed to /joint_states")
        
        # Subscribe to cameras
        if self.robot_config:
            sensors = self.robot_config.get('sensors', [])
            for sensor in sensors:
                if sensor.get('type') == 'camera':
                    self.subscribe_camera(sensor)
        
        # Subscribe to IMU
        self.imu_sub = self.node.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos
        )
        
        # Subscribe to force-torque
        self.ft_sub = self.node.create_subscription(
            WrenchStamped,
            '/force_torque/data',
            self.ft_callback,
            qos
        )
        
        # Service clients
        self.reset_client = self.node.create_client(Empty, '/simulator/reset')
        self.pause_client = self.node.create_client(SetBool, '/simulator/pause')
        
        logger.info("ROS2 communications initialized")
    
    def subscribe_camera(self, sensor_config: Dict):
        """Subscribe to a camera topic."""
        name = sensor_config.get('name')
        topic = sensor_config.get('topic', f'/camera/{name}/image_raw')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        def callback(msg):
            self.camera_callback(msg, name)
        
        sub = self.node.create_subscription(
            Image,
            topic,
            callback,
            qos
        )
        
        self.camera_subs[name] = sub
        logger.info(f"Subscribed to camera: {topic}")
    
    def joint_state_callback(self, msg: 'JointState'):
        """Handle joint state updates from genesis_ros."""
        self.current_joint_state = msg
        
        # Convert to observation for policy
        if self.policy is not None:
            self.current_obs = self.joint_state_to_obs(msg)
    
    def camera_callback(self, msg: 'Image', camera_name: str):
        """Handle camera image updates from genesis_ros."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Store for WebSocket streaming
            self.current_cameras[camera_name] = cv_image
            
            # Stream main camera to WebSocket clients
            if camera_name == 'main_camera':
                asyncio.create_task(self.stream_frame(cv_image))
        
        except Exception as e:
            logger.error(f"Error processing camera frame: {e}")
    
    def imu_callback(self, msg: 'Imu'):
        """Handle IMU updates."""
        self.current_imu = msg
    
    def ft_callback(self, msg: 'WrenchStamped'):
        """Handle force-torque sensor updates."""
        self.current_ft = msg
    
    def joint_state_to_obs(self, joint_state: 'JointState') -> np.ndarray:
        """
        Convert ROS2 JointState to observation vector for policy.
        
        Observation includes:
        - Joint positions
        - Joint velocities
        - (Optional) End-effector pose, forces, etc.
        """
        obs_list = []
        
        # Joint positions
        obs_list.extend(joint_state.position[:self.num_joints])
        
        # Joint velocities
        obs_list.extend(joint_state.velocity[:self.num_joints])
        
        # TODO: Add end-effector pose, goal, etc.
        
        return np.array(obs_list, dtype=np.float32)
    
    async def gamepad_to_joint_commands(self, gamepad_state: Dict):
        """
        Convert gamepad input to joint commands with HIL blending.
        
        Pipeline:
        1. Get teleop action from gamepad
        2. Get policy action (if available)
        3. Blend with ActionRouter
        4. Apply safety clamps
        5. Publish to ROS2
        """
        # Get teleop action from gamepad
        teleop_action = self.teleop_bridge.get_action(joystick_state=gamepad_state)
        
        # Get policy action if available
        policy_action = None
        confidence = None
        if self.config.enable_rl and self.policy is not None and self.current_obs is not None:
            try:
                import torch
                # Convert numpy observation to tensor on correct device
                obs_tensor = torch.from_numpy(self.current_obs).unsqueeze(0).to(self.policy_device)
                with torch.no_grad():
                    policy_action_tensor = self.policy(obs_tensor)
                policy_action = policy_action_tensor.squeeze(0).cpu().numpy()
                confidence = 1.0  # Could extract from policy
            except Exception as e:
                logger.error(f"Policy inference error: {e}")
        
        # Blend actions
        deadman_held = gamepad_state.get('L1', False)
        applied_action, blend_log = self.action_router.blend(
            teleop_action,
            policy_action,
            deadman_held=deadman_held,
            confidence=confidence
        )
        
        # Apply safety clamps
        safe_action, safety_flags = self.action_spec.clamp_action(
            applied_action,
            dt=0.01  # TODO: Get from config
        )
        
        # Check for anomalies
        anomalies = self.anomaly_detector.check_action(safe_action, dt=0.01)
        if self.anomaly_detector.update(anomalies):
            logger.error("Auto-stop triggered by anomaly detector")
            self.action_router.force_alpha_zero(reason="anomaly")
            return
        
        # Publish to ROS2
        if self.joint_cmd_pub:
            msg = Float64MultiArray()
            msg.data = safe_action.tolist()
            self.joint_cmd_pub.publish(msg)
        
        # Update statistics
        self.step_count += 1
        self.fps_counter += 1
        
        # Update FPS
        now = time.time()
        if now - self.fps_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_time = now
    
    async def stream_frame(self, cv_image: np.ndarray):
        """Encode frame as JPEG and stream to WebSocket clients."""
        now = time.time()
        if now - self.last_frame_time < self.frame_interval:
            return

        self.last_frame_time = now

        # Encode as JPEG (GPU if pipeline available, else CPU)
        if self.gpu_pipeline:
            # GPU pipeline expects RGB, cv_image is BGR from ROS
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(rgb_image, self.config.jpeg_quality)
        else:
            _, jpeg_buf = cv2.imencode(
                '.jpg',
                cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
            )
            jpeg_bytes = jpeg_buf.tobytes()

        # Broadcast to all WebSocket clients
        await self.broadcast_frame(jpeg_bytes)
    
    async def broadcast_frame(self, frame_bytes: bytes):
        """Broadcast frame to all connected WebSocket clients."""
        if not self.ws_clients:
            return
        
        await asyncio.gather(
            *[client.send(frame_bytes) for client in self.ws_clients],
            return_exceptions=True
        )
    
    async def handle_websocket(self, websocket):
        """Handle WebSocket client connection for frame streaming."""
        self.ws_clients.add(websocket)
        logger.info(f"WebSocket client connected. Total: {len(self.ws_clients)}")
        
        try:
            while True:
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                except websockets.exceptions.ConnectionClosed:
                    break
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.ws_clients.discard(websocket)
            logger.info(f"WebSocket client disconnected. Total: {len(self.ws_clients)}")
    
    def setup_socketio(self):
        """Set up Socket.io client for SDR_OS communication."""
        if socketio is None:
            logger.warning("Socket.io not available")
            return
        
        self.sio = socketio.AsyncClient(
            logger=False,
            engineio_logger=False,
            reconnection=True,
            reconnection_attempts=0,
            reconnection_delay=1.0,
            reconnection_delay_max=5.0,
            engineio_options={
                "ping_interval": 25,
                "ping_timeout": 60
            }
        )
        
        @self.sio.on('connect')
        async def on_connect():
            logger.info("Connected to SDR_OS server")
            await self.sio.emit('genesis_identify', {
                'type': 'genesis_ros_bridge',
                'robot': self.robot_config.get('robot', {}).get('name') if self.robot_config else None
            })
        
        @self.sio.on('disconnect')
        async def on_disconnect():
            logger.info("Disconnected from SDR_OS server")
        
        @self.sio.on('controller_joystick_state')
        async def on_joystick(data):
            await self.gamepad_to_joint_commands(data)
        
        @self.sio.on('genesis_reset')
        async def on_reset(data):
            logger.info("Reset requested")
            if self.reset_client and self.reset_client.service_is_ready():
                future = self.reset_client.call_async(Empty.Request())
                # Don't await here, handle in separate task
            self.action_router.reset()
            self.action_spec.reset()
        
        @self.sio.on('genesis_pause')
        async def on_pause(data):
            paused = data.get('paused', True)
            logger.info(f"Pause requested: {paused}")
            if self.pause_client and self.pause_client.service_is_ready():
                request = SetBool.Request()
                request.data = paused
                future = self.pause_client.call_async(request)
        
        @self.sio.on('genesis_set_control_mode')
        async def on_set_control_mode(data):
            mode = data.get('mode', 'cartesian')
            logger.info(f"Control mode set to: {mode}")
            self.config.control_mode = mode
    
    def load_policy(self, checkpoint_path: str):
        """Load RL policy from checkpoint."""
        try:
            import torch
            from rl.utils.device import get_device

            self.policy_device = get_device("auto")
            checkpoint = torch.load(checkpoint_path, map_location=self.policy_device)

            # Handle both full model saves and state_dict saves
            if isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
                from rl.policies.bc_policy import BCPolicy
                obs_dim = checkpoint.get('obs_dim', 48)
                action_dim = checkpoint.get('action_dim', 12)
                self.policy = BCPolicy(obs_dim, action_dim)
                self.policy.load_state_dict(checkpoint['model_state_dict'])
            else:
                self.policy = checkpoint

            self.policy = self.policy.to(self.policy_device)
            self.policy.eval()
            logger.info(f"Policy loaded from {checkpoint_path} on {self.policy_device}")
        except Exception as e:
            logger.error(f"Failed to load policy: {e}")
            self.policy = None
            self.policy_device = None
    
    async def ros2_spin_loop(self):
        """Spin ROS2 node in asyncio loop."""
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            await asyncio.sleep(0.001)
    
    async def run(self):
        """Run the bridge server."""
        logger.info("=" * 60)
        logger.info("Starting Genesis-ROS Bridge for SDR_OS")
        logger.info("=" * 60)
        
        # Initialize ROS2
        self.initialize_ros2()
        
        # Load policy if specified
        if self.config.policy_checkpoint:
            self.load_policy(self.config.policy_checkpoint)
        
        # Setup Socket.io
        self.setup_socketio()
        
        # Connect to SDR_OS
        if self.sio:
            try:
                await self.sio.connect(self.config.sdr_os_url)
            except Exception as e:
                logger.error(f"Failed to connect to SDR_OS: {e}")
        
        # Start ROS2 spin loop
        ros2_task = asyncio.create_task(self.ros2_spin_loop())
        
        # Start WebSocket server
        if websockets and ws_serve:
            async with ws_serve(self.handle_websocket, self.config.ws_host, self.config.ws_port):
                logger.info(f"WebSocket server listening on {self.config.ws_host}:{self.config.ws_port}")
                logger.info("Genesis-ROS Bridge ready")
                
                # Run forever
                await asyncio.Event().wait()
        else:
            logger.warning("WebSocket not available, running without frame streaming")
            await asyncio.Event().wait()
    
    def shutdown(self):
        """Clean shutdown."""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        logger.info("Genesis-ROS Bridge shutdown complete")


async def main():
    """Entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Genesis-ROS Bridge for SDR_OS")
    parser.add_argument("--config", type=str, help="Path to robot YAML config")
    parser.add_argument("--ws-port", type=int, default=9091, help="WebSocket port")
    parser.add_argument("--sdr-url", type=str, default="http://localhost:3000", help="SDR_OS URL")
    parser.add_argument("--control-mode", type=str, default="joint_velocity", choices=['joint_velocity', 'cartesian_velocity'])
    parser.add_argument("--policy", type=str, help="Path to policy checkpoint")
    parser.add_argument("--no-rl", action="store_true", help="Disable RL policy")
    args = parser.parse_args()
    
    config = BridgeConfig(
        robot_config_path=args.config,
        ws_port=args.ws_port,
        sdr_os_url=args.sdr_url,
        control_mode=args.control_mode,
        enable_rl=not args.no_rl,
        policy_checkpoint=args.policy
    )
    
    bridge = GenesisROSBridge(config)
    
    try:
        await bridge.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
