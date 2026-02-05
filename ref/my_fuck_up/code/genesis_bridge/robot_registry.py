"""
Robot Registry for Genesis Bridge

Manages multiple robot URDFs and allows dynamic switching.
Extended to support genesis_ros YAML configuration loading.
Includes memory estimation for VRAM/DRAM requirements.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Type
from pathlib import Path
import logging
import yaml

# Handle both relative and absolute imports
try:
    from .memory_estimator import MemoryEstimator, estimate_robot_memory
except ImportError:
    from memory_estimator import MemoryEstimator, estimate_robot_memory

logger = logging.getLogger(__name__)


@dataclass
class RobotConfig:
    """Configuration for a registered robot."""
    name: str
    urdf_path: str
    label: Optional[str] = None  # Human-readable name
    description: Optional[str] = None
    category: Optional[str] = None  # manipulator, mobile, humanoid, quadruped
    initial_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.5])
    initial_orientation: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    env_class: Optional[Type] = None  # Optional ManagedEnvironment subclass
    env_kwargs: Dict[str, Any] = field(default_factory=dict)
    action_scale: float = 0.25
    default_joint_positions: Optional[Dict[str, float]] = None
    gs_ros_config: Optional[str] = None  # Path to genesis_ros YAML config
    thumbnail: Optional[str] = None  # Path to thumbnail image
    specs: Optional[Dict[str, Any]] = None  # Robot specifications (DOF, mass, payload, etc.)
    memory_estimate: Optional[Dict[str, Any]] = None  # VRAM/DRAM estimates
    
    def to_dict(self) -> Dict:
        """Convert to JSON-serializable dict."""
        return {
            "name": self.name,
            "label": self.label or self.name,
            "description": self.description or "",
            "category": self.category or "unknown",
            "urdf_path": self.urdf_path,
            "initial_position": self.initial_position,
            "initial_orientation": self.initial_orientation,
            "thumbnail": self.thumbnail,
            "specs": self.specs or {},
            "gs_ros_config": self.gs_ros_config,
            "memory_estimate": self.memory_estimate
        }


class RobotRegistry:
    """
    Registry for managing multiple robot configurations.
    
    Usage:
        registry = RobotRegistry()
        registry.register("go2", "/path/to/go2.urdf", label="Unitree Go2")
        registry.register("spot", "/path/to/spot.urdf", label="Boston Dynamics Spot")
        
        robots = registry.get_robot_list()
        config = registry.get_robot("go2")
    """
    
    def __init__(self):
        self._robots: Dict[str, RobotConfig] = {}
        self._current_robot: Optional[str] = None
        
        # Register default robots if available
        self._register_default_robots()
    
    def _register_default_robots(self):
        """Register any default/bundled robots."""
        # Check for common URDF locations
        urdf_paths = [
            Path("/home/ethan/dev/Genesis/urdf"),
            Path.home() / "robots" / "urdf",
            Path(__file__).parent.parent / "urdf"
        ]
        
        for urdf_dir in urdf_paths:
            if urdf_dir.exists():
                # Auto-discover URDFs
                for urdf_file in urdf_dir.glob("**/*.urdf"):
                    name = urdf_file.stem
                    if name not in self._robots:
                        self.register(
                            name=name,
                            urdf_path=str(urdf_file),
                            label=name.replace("_", " ").title()
                        )
    
    def register(
        self,
        name: str,
        urdf_path: str,
        label: Optional[str] = None,
        description: Optional[str] = None,
        category: Optional[str] = None,
        initial_position: Optional[List[float]] = None,
        initial_orientation: Optional[List[float]] = None,
        env_class: Optional[Type] = None,
        env_kwargs: Optional[Dict[str, Any]] = None,
        action_scale: float = 0.25,
        default_joint_positions: Optional[Dict[str, float]] = None,
        gs_ros_config: Optional[str] = None,
        thumbnail: Optional[str] = None,
        specs: Optional[Dict[str, Any]] = None,
        base_path: Optional[str] = None
    ) -> None:
        """
        Register a robot configuration.
        
        Args:
            name: Unique identifier for the robot
            urdf_path: Path to URDF file
            label: Human-readable name (optional)
            description: Description text (optional)
            category: Robot category (manipulator, mobile, humanoid, quadruped)
            initial_position: [x, y, z] spawn position
            initial_orientation: [w, x, y, z] quaternion
            env_class: Optional ManagedEnvironment subclass
            env_kwargs: Kwargs to pass to env_class
            action_scale: Action scaling factor
            default_joint_positions: Dict of joint names to default positions
            gs_ros_config: Path to genesis_ros YAML config
            thumbnail: Path to thumbnail image
            specs: Robot specifications dict
            base_path: Base path for resolving relative URDF paths
        """
        # Validate URDF path
        urdf_file = Path(urdf_path)
        if not urdf_file.exists() and not urdf_path.startswith("${"):
            logger.warning(f"URDF file not found: {urdf_path}")
        
        # Calculate memory estimate
        memory_estimate = None
        try:
            memory_estimate = estimate_robot_memory(
                urdf_path,
                scene_config=None,
                base_path=base_path
            )
            logger.debug(f"Memory estimate for {name}: VRAM={memory_estimate.get('vram_mb', 0):.1f}MB, DRAM={memory_estimate.get('dram_mb', 0):.1f}MB")
        except Exception as e:
            logger.warning(f"Failed to estimate memory for {name}: {e}")
        
        config = RobotConfig(
            name=name,
            urdf_path=urdf_path,
            label=label,
            description=description,
            category=category,
            initial_position=initial_position or [0.0, 0.0, 0.5],
            initial_orientation=initial_orientation or [1.0, 0.0, 0.0, 0.0],
            env_class=env_class,
            env_kwargs=env_kwargs or {},
            action_scale=action_scale,
            default_joint_positions=default_joint_positions,
            gs_ros_config=gs_ros_config,
            thumbnail=thumbnail,
            specs=specs,
            memory_estimate=memory_estimate
        )
        
        self._robots[name] = config
        logger.info(f"Registered robot: {name} ({label or name})")
    
    def unregister(self, name: str) -> bool:
        """Remove a robot from the registry."""
        if name in self._robots:
            del self._robots[name]
            if self._current_robot == name:
                self._current_robot = None
            logger.info(f"Unregistered robot: {name}")
            return True
        return False
    
    def get_robot(self, name: str) -> Optional[RobotConfig]:
        """Get robot configuration by name."""
        return self._robots.get(name)
    
    def get_robot_list(self) -> List[Dict]:
        """Get list of all registered robots (for frontend dropdown)."""
        return [config.to_dict() for config in self._robots.values()]
    
    def get_robot_names(self) -> List[str]:
        """Get list of robot names."""
        return list(self._robots.keys())
    
    @property
    def current_robot(self) -> Optional[str]:
        """Get currently active robot name."""
        return self._current_robot
    
    @current_robot.setter
    def current_robot(self, name: Optional[str]):
        """Set currently active robot."""
        if name is not None and name not in self._robots:
            raise ValueError(f"Unknown robot: {name}")
        self._current_robot = name
    
    def get_current_config(self) -> Optional[RobotConfig]:
        """Get configuration for currently active robot."""
        if self._current_robot:
            return self._robots.get(self._current_robot)
        return None
    
    def __len__(self) -> int:
        return len(self._robots)
    
    def __contains__(self, name: str) -> bool:
        return name in self._robots
    
    def __iter__(self):
        return iter(self._robots.values())


class GenesisROSRobotRegistry(RobotRegistry):
    """
    Extended registry that loads robots from genesis_ros YAML configs.
    
    This automatically discovers robot configurations from the genesis_ros
    configs directory and registers them.
    """
    
    def __init__(self, gs_ros_config_path: str):
        """
        Initialize registry and load genesis_ros configs.
        
        Args:
            gs_ros_config_path: Path to genesis_ros configs directory
        """
        self.gs_ros_config_path = Path(gs_ros_config_path)
        super().__init__()
        self._load_gs_ros_robots()
    
    def _load_gs_ros_robots(self):
        """Auto-discover and register robots from genesis_ros configs."""
        if not self.gs_ros_config_path.exists():
            logger.warning(f"genesis_ros config path not found: {self.gs_ros_config_path}")
            return
        
        # Look for robot configs in robots/ subdirectory
        robots_dir = self.gs_ros_config_path / "robots"
        if not robots_dir.exists():
            logger.warning(f"No robots directory found: {robots_dir}")
            return
        
        # Base path for resolving relative URDF paths (parent of configs dir)
        base_path = str(self.gs_ros_config_path.parent)
        
        robot_configs = list(robots_dir.glob("*.yaml"))
        logger.info(f"Found {len(robot_configs)} robot configs in {robots_dir}")
        
        for config_file in robot_configs:
            try:
                with open(config_file) as f:
                    config = yaml.safe_load(f)
                
                # Extract robot info
                robot_info = config.get('robot', {})
                name = robot_info.get('name', config_file.stem)
                
                # Extract specs
                specs = robot_info.get('specs', {})
                
                # Get scene config for memory estimation
                scene_config = config.get('scene', {})
                
                # Register robot with base_path for memory estimation
                self.register(
                    name=name,
                    urdf_path=robot_info.get('urdf_path', ''),
                    label=robot_info.get('label', name.replace('_', ' ').title()),
                    description=robot_info.get('description', ''),
                    category=robot_info.get('category', 'unknown'),
                    initial_position=robot_info.get('initial_position', [0, 0, 0]),
                    initial_orientation=robot_info.get('initial_orientation', [1, 0, 0, 0]),
                    gs_ros_config=str(config_file),
                    specs=specs,
                    base_path=base_path
                )
                
                logger.info(f"Loaded genesis_ros robot config: {name}")
                
            except Exception as e:
                logger.error(f"Failed to load robot config {config_file}: {e}")
    
    def get_robot_config_yaml(self, name: str) -> Optional[Dict]:
        """
        Get full YAML configuration for a robot.
        
        Args:
            name: Robot name
        
        Returns:
            Full YAML config dict or None
        """
        robot = self.get_robot(name)
        if not robot or not robot.gs_ros_config:
            return None
        
        try:
            with open(robot.gs_ros_config) as f:
                return yaml.safe_load(f)
        except Exception as e:
            logger.error(f"Failed to load YAML for {name}: {e}")
            return None


# Global registry instance
_global_registry: Optional[RobotRegistry] = None


def get_registry() -> RobotRegistry:
    """Get the global robot registry instance."""
    global _global_registry
    if _global_registry is None:
        _global_registry = RobotRegistry()
    return _global_registry


def get_genesis_ros_registry(config_path: str) -> GenesisROSRobotRegistry:
    """
    Get or create genesis_ros registry.
    
    Args:
        config_path: Path to genesis_ros configs directory
    
    Returns:
        GenesisROSRobotRegistry instance
    """
    global _global_registry
    if not isinstance(_global_registry, GenesisROSRobotRegistry):
        _global_registry = GenesisROSRobotRegistry(config_path)
    return _global_registry


def register_robot(*args, **kwargs) -> None:
    """Convenience function to register a robot in the global registry."""
    get_registry().register(*args, **kwargs)
