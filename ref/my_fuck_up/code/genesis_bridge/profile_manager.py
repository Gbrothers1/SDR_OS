"""
Profile Manager for SDR_OS

Manages configuration profiles for switching between simulation and real robot modes.
Profiles define robot, control, safety, and training settings.
"""

import yaml
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class Profile:
    """Configuration profile."""
    name: str
    label: str
    description: str
    mode: str  # simulation, real_robot
    config: Dict
    
    def to_dict(self) -> Dict:
        """Convert to JSON-serializable dict."""
        return {
            "name": self.name,
            "label": self.label,
            "description": self.description,
            "mode": self.mode
        }


class ProfileManager:
    """
    Manages configuration profiles for SDR_OS.
    
    Profiles allow quick switching between:
    - Simulation vs real robot
    - Different robots
    - Training vs evaluation modes
    - Safety configurations
    """
    
    def __init__(self, profiles_dir: str = "configs/profiles"):
        """
        Initialize profile manager.
        
        Args:
            profiles_dir: Directory containing profile YAML files
        """
        self.profiles_dir = Path(profiles_dir)
        self.profiles: Dict[str, Profile] = {}
        self.current_profile: Optional[str] = None
        
        # Load profiles
        self.load_profiles()
    
    def load_profiles(self):
        """Load all profiles from profiles directory."""
        if not self.profiles_dir.exists():
            logger.warning(f"Profiles directory not found: {self.profiles_dir}")
            return
        
        yaml_files = list(self.profiles_dir.glob("*.yaml"))
        logger.info(f"Loading {len(yaml_files)} profiles from {self.profiles_dir}")
        
        for yaml_file in yaml_files:
            try:
                with open(yaml_file) as f:
                    config = yaml.safe_load(f)
                
                # Extract profile metadata
                profile_info = config.get('profile', {})
                name = profile_info.get('name', yaml_file.stem)
                
                profile = Profile(
                    name=name,
                    label=profile_info.get('label', name),
                    description=profile_info.get('description', ''),
                    mode=profile_info.get('mode', 'simulation'),
                    config=config
                )
                
                self.profiles[name] = profile
                logger.info(f"Loaded profile: {name} ({profile.label})")
                
            except Exception as e:
                logger.error(f"Failed to load profile {yaml_file}: {e}")
    
    def get_profile(self, name: str) -> Optional[Profile]:
        """Get profile by name."""
        return self.profiles.get(name)
    
    def get_profile_list(self) -> List[Dict]:
        """Get list of available profiles."""
        return [p.to_dict() for p in self.profiles.values()]
    
    def get_profile_config(self, name: str) -> Optional[Dict]:
        """Get full configuration for a profile."""
        profile = self.get_profile(name)
        return profile.config if profile else None
    
    def set_current_profile(self, name: str) -> bool:
        """
        Set current active profile.
        
        Args:
            name: Profile name
        
        Returns:
            True if successful
        """
        if name not in self.profiles:
            logger.error(f"Profile not found: {name}")
            return False
        
        self.current_profile = name
        logger.info(f"Active profile set to: {name}")
        return True
    
    def get_current_profile(self) -> Optional[Profile]:
        """Get currently active profile."""
        if self.current_profile:
            return self.profiles.get(self.current_profile)
        return None
    
    def apply_profile(self, name: str) -> Dict:
        """
        Apply a profile and return its configuration.
        
        This extracts key settings from the profile for easy access.
        
        Args:
            name: Profile name
        
        Returns:
            Dictionary with extracted settings
        """
        profile = self.get_profile(name)
        if not profile:
            raise ValueError(f"Profile not found: {name}")
        
        config = profile.config
        
        # Extract commonly used settings
        settings = {
            "profile_name": name,
            "mode": profile.mode,
            "robot_name": config.get("robot", {}).get("name"),
            "robot_config_path": config.get("robot", {}).get("config_path"),
            "backend_type": config.get("backend", {}).get("type"),
            "control_mode": config.get("control", {}).get("mode"),
            "rl_enabled": config.get("rl", {}).get("enabled", False),
            "safety_enabled": config.get("safety", {}).get("enabled", True),
            "ws_port": config.get("visualization", {}).get("ws_port", 9091),
            "full_config": config
        }
        
        self.set_current_profile(name)
        return settings
    
    def create_profile(
        self,
        name: str,
        label: str,
        mode: str,
        config: Dict,
        save: bool = True
    ) -> Profile:
        """
        Create a new profile.
        
        Args:
            name: Unique profile name
            label: Human-readable label
            mode: Profile mode (simulation, real_robot)
            config: Full configuration dictionary
            save: Whether to save to disk
        
        Returns:
            Created Profile object
        """
        profile = Profile(
            name=name,
            label=label,
            description=config.get('profile', {}).get('description', ''),
            mode=mode,
            config=config
        )
        
        self.profiles[name] = profile
        
        if save:
            self.save_profile(profile)
        
        logger.info(f"Created profile: {name}")
        return profile
    
    def save_profile(self, profile: Profile):
        """Save profile to YAML file."""
        output_path = self.profiles_dir / f"{profile.name}.yaml"
        
        # Ensure directory exists
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(output_path, 'w') as f:
                yaml.dump(profile.config, f, default_flow_style=False, sort_keys=False)
            
            logger.info(f"Saved profile to: {output_path}")
        except Exception as e:
            logger.error(f"Failed to save profile: {e}")
    
    def delete_profile(self, name: str) -> bool:
        """
        Delete a profile.
        
        Args:
            name: Profile name
        
        Returns:
            True if successful
        """
        if name not in self.profiles:
            return False
        
        # Remove from memory
        del self.profiles[name]
        
        # Delete file
        profile_file = self.profiles_dir / f"{name}.yaml"
        if profile_file.exists():
            profile_file.unlink()
        
        # Clear current if deleted
        if self.current_profile == name:
            self.current_profile = None
        
        logger.info(f"Deleted profile: {name}")
        return True
    
    def get_profiles_by_mode(self, mode: str) -> List[Profile]:
        """
        Get all profiles for a specific mode.
        
        Args:
            mode: Mode filter (simulation, real_robot)
        
        Returns:
            List of matching profiles
        """
        return [p for p in self.profiles.values() if p.mode == mode]
    
    def get_profiles_by_robot(self, robot_name: str) -> List[Profile]:
        """
        Get all profiles for a specific robot.
        
        Args:
            robot_name: Robot name
        
        Returns:
            List of matching profiles
        """
        return [
            p for p in self.profiles.values()
            if p.config.get("robot", {}).get("name") == robot_name
        ]


# Global profile manager instance
_profile_manager: Optional[ProfileManager] = None


def get_profile_manager(profiles_dir: str = "configs/profiles") -> ProfileManager:
    """
    Get global profile manager instance.
    
    Args:
        profiles_dir: Profiles directory (only used on first call)
    
    Returns:
        ProfileManager instance
    """
    global _profile_manager
    if _profile_manager is None:
        _profile_manager = ProfileManager(profiles_dir)
    return _profile_manager
