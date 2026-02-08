"""
ROS2 node for the Genesis bridge server.

Provides:
- /cmd_vel subscriber: receives velocity commands from ROS
- /cmd_vel publisher: echoes gamepad commands to ROS
- /set_command_source service: switches between gamepad and ros input
- /go2_debug publisher: publishes debug status as JSON string

Two backends:
1. rclpy (direct ROS2, when running inside Docker)
2. roslibpy (WebSocket via rosbridge at ws://localhost:9090, when running on host)
"""

import json
import logging
import threading
import time

logger = logging.getLogger("genesis_bridge.ros_node")

# Try rclpy first (fastest, direct DDS)
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

# Fallback: roslibpy (pure Python WebSocket client for rosbridge)
try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False

ROS_AVAILABLE = RCLPY_AVAILABLE or ROSLIBPY_AVAILABLE

if not ROS_AVAILABLE:
    logger.warning("Neither rclpy nor roslibpy available — ROS integration disabled")
elif RCLPY_AVAILABLE:
    logger.info("ROS backend: rclpy (direct)")
else:
    logger.info("ROS backend: roslibpy (rosbridge WebSocket)")


class GenesisBridgeROSNode:
    """Manages ROS2 communication for the Genesis bridge server.

    Supports two backends:
    - rclpy: direct ROS2 node (when running in Docker with ROS installed)
    - roslibpy: connects to rosbridge_server WebSocket (when running on host)
    """

    def __init__(self, bridge_server, rosbridge_host="localhost", rosbridge_port=9090):
        self.bridge = bridge_server
        self.node = None
        self._spin_thread = None
        self._running = False
        self._cmd_vel_pub = None
        self._debug_pub = None
        self._backend = None
        self._rosbridge_host = rosbridge_host
        self._rosbridge_port = rosbridge_port
        # roslibpy-specific
        self._ros_client = None
        self._cmd_vel_sub = None

    def start(self):
        """Initialize ROS communication and start background thread."""
        if RCLPY_AVAILABLE:
            return self._start_rclpy()
        elif ROSLIBPY_AVAILABLE:
            return self._start_roslibpy()
        else:
            logger.warning("Cannot start ROS node: no ROS backend available")
            return False

    def _start_rclpy(self):
        """Start using native rclpy backend."""
        try:
            rclpy.init()
            self.node = rclpy.create_node("genesis_bridge")

            self.node.create_subscription(
                Twist, "/cmd_vel", self._cmd_vel_callback_rclpy, 10
            )
            self._cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
            self._debug_pub = self.node.create_publisher(String, "/go2_debug", 10)
            self.node.create_service(
                SetBool, "/set_command_source", self._set_source_callback
            )
            self.node.create_timer(0.1, self._publish_debug_rclpy)

            self._backend = "rclpy"
            self._running = True
            self._spin_thread = threading.Thread(target=self._spin_rclpy, daemon=True)
            self._spin_thread.start()

            logger.info("ROS2 node 'genesis_bridge' started (rclpy)")
            return True
        except Exception as e:
            logger.error(f"Failed to start ROS node (rclpy): {e}")
            return False

    def _start_roslibpy(self):
        """Start using roslibpy WebSocket backend.

        Uses roslibpy's non-blocking run() which starts the Twisted reactor
        in a daemon thread with installSignalHandlers=False (avoids the
        'signal only works in main thread' error from run_forever()).
        """
        try:
            self._running = True

            client = roslibpy.Ros(
                host=self._rosbridge_host, port=self._rosbridge_port
            )

            # run() starts reactor in a background thread and waits for connection
            try:
                client.run(timeout=10)
            except Exception as e:
                logger.error(
                    f"Failed to connect to rosbridge at "
                    f"ws://{self._rosbridge_host}:{self._rosbridge_port}: {e}"
                )
                self._running = False
                return False

            if not client.is_connected:
                logger.error(
                    f"Failed to connect to rosbridge at "
                    f"ws://{self._rosbridge_host}:{self._rosbridge_port}"
                )
                self._running = False
                return False

            self._ros_client = client
            self._cmd_vel_pub = roslibpy.Topic(
                client, "/cmd_vel", "geometry_msgs/msg/Twist"
            )
            self._debug_pub = roslibpy.Topic(
                client, "/go2_debug", "std_msgs/msg/String"
            )
            self._cmd_vel_sub = roslibpy.Topic(
                client, "/cmd_vel", "geometry_msgs/msg/Twist"
            )
            self._cmd_vel_sub.subscribe(self._cmd_vel_callback_roslibpy)
            self._backend = "roslibpy"

            # Start debug publisher thread
            self._spin_thread = threading.Thread(target=self._spin_roslibpy, daemon=True)
            self._spin_thread.start()

            logger.info(
                f"ROS bridge connected via roslibpy "
                f"(ws://{self._rosbridge_host}:{self._rosbridge_port})"
            )
            return True
        except Exception as e:
            logger.error(f"Failed to start ROS node (roslibpy): {e}")
            self._ros_client = None
            return False

    def stop(self):
        """Shutdown ROS communication."""
        self._running = False
        if self._backend == "rclpy":
            if self.node is not None:
                self.node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        elif self._backend == "roslibpy":
            if self._cmd_vel_sub is not None:
                self._cmd_vel_sub.unsubscribe()
            if self._ros_client is not None:
                try:
                    self._ros_client.terminate()
                except Exception:
                    pass

    def publish_cmd_vel(self, lin_x: float, lin_y: float, ang_z: float):
        """Publish gamepad velocity to /cmd_vel for external visibility."""
        if self._cmd_vel_pub is None:
            return

        if self._backend == "rclpy":
            msg = Twist()
            msg.linear.x = lin_x
            msg.linear.y = lin_y
            msg.angular.z = ang_z
            self._cmd_vel_pub.publish(msg)
        elif self._backend == "roslibpy":
            self._cmd_vel_pub.publish(roslibpy.Message({
                "linear": {"x": lin_x, "y": lin_y, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": ang_z},
            }))

    # --- rclpy callbacks ---

    def _cmd_vel_callback_rclpy(self, msg):
        """Handle /cmd_vel via rclpy."""
        if self.bridge.command_source != "ros":
            return
        if self.bridge.forge_env is None:
            return

        import torch
        import genesis as gs

        lin_x = max(-1.0, min(1.0, msg.linear.x))
        lin_y = max(-0.5, min(0.5, msg.linear.y))
        ang_z = max(-1.0, min(1.0, msg.angular.z))

        vcm = self.bridge.forge_env.velocity_command
        vcm.set_command("lin_vel_x", torch.tensor(lin_x, device=gs.device))
        vcm.set_command("lin_vel_y", torch.tensor(lin_y, device=gs.device))
        vcm.set_command("ang_vel_z", torch.tensor(ang_z, device=gs.device))

    def _set_source_callback(self, request, response):
        """ROS service: SetBool where True=ros, False=gamepad."""
        new_source = "ros" if request.data else "gamepad"
        old_source = self.bridge.command_source
        self.bridge.command_source = new_source

        if self.bridge.forge_env is not None and old_source != new_source:
            self.bridge.forge_env.set_velocity_from_gamepad({
                'leftStickX': 0, 'leftStickY': 0,
                'rightStickX': 0, 'rightStickY': 0,
            })

        response.success = True
        response.message = f"Command source: {old_source} -> {new_source}"
        logger.info(response.message)
        return response

    def _publish_debug_rclpy(self):
        """Publish debug status via rclpy."""
        if self._debug_pub is None:
            return
        msg = String()
        msg.data = json.dumps(self._get_debug_status())
        self._debug_pub.publish(msg)

    def _spin_rclpy(self):
        """Background thread for rclpy spinning."""
        while self._running:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                break

    # --- roslibpy callbacks ---

    def _cmd_vel_callback_roslibpy(self, message):
        """Handle /cmd_vel via roslibpy."""
        if self.bridge.command_source != "ros":
            return
        if self.bridge.forge_env is None:
            return

        import torch
        import genesis as gs

        linear = message.get("linear", {})
        angular = message.get("angular", {})

        lin_x = max(-1.0, min(1.0, float(linear.get("x", 0.0))))
        lin_y = max(-0.5, min(0.5, float(linear.get("y", 0.0))))
        ang_z = max(-1.0, min(1.0, float(angular.get("z", 0.0))))

        vcm = self.bridge.forge_env.velocity_command
        vcm.set_command("lin_vel_x", torch.tensor(lin_x, device=gs.device))
        vcm.set_command("lin_vel_y", torch.tensor(lin_y, device=gs.device))
        vcm.set_command("ang_vel_z", torch.tensor(ang_z, device=gs.device))

    def _spin_roslibpy(self):
        """Background thread for roslibpy debug publishing."""
        while self._running:
            try:
                if self._debug_pub is not None and self._ros_client and self._ros_client.is_connected:
                    self._debug_pub.publish(roslibpy.Message({
                        "data": json.dumps(self._get_debug_status())
                    }))
                time.sleep(0.1)
            except Exception:
                break

    # --- shared helpers ---

    def _get_debug_status(self) -> dict:
        """Build debug status dict."""
        status = {
            "command_source": self.bridge.command_source,
            "deadman_active": self.bridge.deadman_active,
            "policy_loaded": self.bridge.policy is not None,
            "policy_checkpoint": self.bridge.policy_checkpoint_path,
            "paused": self.bridge.paused,
            "step_count": self.bridge.step_count,
            "fps": self.bridge.fps,
        }
        if self.bridge.forge_env is not None:
            status["velocity_command"] = self.bridge.forge_env.get_velocity_command()
        return status
