"""
Memory Estimator for Genesis Robots

Estimates VRAM and DRAM requirements based on:
- Mesh vertex/polygon count from OBJ/DAE/STL files
- Texture sizes (PNG/JPG dimensions * bytes per pixel)
- Number of articulated joints
- Scene objects and their complexity
"""

import os
import re
import struct
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from xml.etree import ElementTree as ET

logger = logging.getLogger(__name__)


@dataclass
class MeshStats:
    """Statistics for a single mesh file."""
    path: str
    vertices: int = 0
    triangles: int = 0
    file_size_bytes: int = 0
    format: str = ""


@dataclass
class TextureStats:
    """Statistics for a texture file."""
    path: str
    width: int = 0
    height: int = 0
    channels: int = 4  # Assume RGBA
    file_size_bytes: int = 0


@dataclass
class MemoryEstimate:
    """Complete memory estimate for a robot/scene."""
    vram_mb: float = 0.0
    dram_mb: float = 0.0
    breakdown: Dict[str, float] = field(default_factory=dict)
    mesh_count: int = 0
    texture_count: int = 0
    joint_count: int = 0
    total_vertices: int = 0
    total_triangles: int = 0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to JSON-serializable dict."""
        return {
            "vram_mb": round(self.vram_mb, 2),
            "dram_mb": round(self.dram_mb, 2),
            "breakdown": {k: round(v, 2) for k, v in self.breakdown.items()},
            "mesh_count": self.mesh_count,
            "texture_count": self.texture_count,
            "joint_count": self.joint_count,
            "total_vertices": self.total_vertices,
            "total_triangles": self.total_triangles
        }


class MemoryEstimator:
    """
    Estimates GPU and system memory requirements for robot simulations.
    
    Memory factors:
    - Mesh vertices: ~32 bytes/vertex (position + normal + UV)
    - Mesh indices: ~4 bytes/index (uint32)
    - Textures: width * height * 4 bytes (RGBA) for VRAM
    - Physics bodies: ~1-2 KB per rigid body
    - Articulations: ~0.5 KB per joint
    - Safety margin: 1.5x multiplier for GPU buffers
    """
    
    # Memory per vertex (position vec3 + normal vec3 + UV vec2 = 32 bytes)
    BYTES_PER_VERTEX = 32
    
    # Memory per triangle index (3 * uint32)
    BYTES_PER_TRIANGLE = 12
    
    # Memory per physics body (approximate)
    BYTES_PER_RIGID_BODY = 2048  # 2 KB
    
    # Memory per joint/articulation
    BYTES_PER_JOINT = 512  # 0.5 KB
    
    # GPU buffer overhead multiplier
    GPU_BUFFER_OVERHEAD = 1.5
    
    # Base GPU memory for Genesis runtime (rendering, physics)
    BASE_VRAM_MB = 256
    
    # Base system RAM for Genesis runtime
    BASE_DRAM_MB = 128
    
    def __init__(self, base_path: Optional[str] = None):
        """
        Initialize the memory estimator.
        
        Args:
            base_path: Base path for resolving relative URDF/asset paths
        """
        self.base_path = Path(base_path) if base_path else Path.cwd()
    
    def estimate_robot_memory(
        self,
        urdf_path: str,
        scene_config: Optional[Dict] = None
    ) -> MemoryEstimate:
        """
        Estimate VRAM and DRAM requirements for a robot.
        
        Args:
            urdf_path: Path to URDF file (relative or absolute)
            scene_config: Optional scene configuration dict
        
        Returns:
            MemoryEstimate with VRAM/DRAM requirements
        """
        estimate = MemoryEstimate()
        
        # Resolve URDF path
        if not os.path.isabs(urdf_path):
            urdf_path = str(self.base_path / urdf_path)
        
        urdf_file = Path(urdf_path)
        if not urdf_file.exists():
            logger.warning(f"URDF file not found: {urdf_path}")
            # Return base estimates
            estimate.vram_mb = self.BASE_VRAM_MB
            estimate.dram_mb = self.BASE_DRAM_MB
            return estimate
        
        # Parse URDF
        try:
            meshes, textures, joint_count = self._parse_urdf(urdf_file)
        except Exception as e:
            logger.error(f"Error parsing URDF: {e}")
            estimate.vram_mb = self.BASE_VRAM_MB
            estimate.dram_mb = self.BASE_DRAM_MB
            return estimate
        
        estimate.joint_count = joint_count
        estimate.mesh_count = len(meshes)
        estimate.texture_count = len(textures)
        
        # Calculate mesh memory
        mesh_vram = 0.0
        mesh_dram = 0.0
        for mesh in meshes:
            estimate.total_vertices += mesh.vertices
            estimate.total_triangles += mesh.triangles
            
            vertex_bytes = mesh.vertices * self.BYTES_PER_VERTEX
            index_bytes = mesh.triangles * self.BYTES_PER_TRIANGLE
            
            mesh_bytes = vertex_bytes + index_bytes
            mesh_vram += mesh_bytes
            mesh_dram += mesh_bytes
        
        # Calculate texture memory (primarily VRAM)
        texture_vram = 0.0
        for tex in textures:
            # RGBA texture memory
            tex_bytes = tex.width * tex.height * tex.channels
            texture_vram += tex_bytes
        
        # Calculate physics memory
        # Each link is roughly a rigid body
        physics_dram = (len(meshes) + 1) * self.BYTES_PER_RIGID_BODY
        
        # Calculate articulation memory
        joint_dram = joint_count * self.BYTES_PER_JOINT
        
        # Scene objects memory (if provided)
        scene_vram = 0.0
        scene_dram = 0.0
        if scene_config:
            objects = scene_config.get('objects', [])
            for obj in objects:
                # Approximate memory for scene objects
                if obj.get('type') == 'box':
                    scene_vram += 1024  # 1 KB for simple geometry
                    scene_dram += 2048
                elif obj.get('type') == 'mesh':
                    # Estimate from file size if available
                    scene_vram += 10240  # 10 KB default
                    scene_dram += 20480
        
        # Apply GPU buffer overhead
        total_vram = (mesh_vram + texture_vram + scene_vram) * self.GPU_BUFFER_OVERHEAD
        total_dram = mesh_dram + physics_dram + joint_dram + scene_dram
        
        # Add base runtime memory
        estimate.vram_mb = (total_vram / (1024 * 1024)) + self.BASE_VRAM_MB
        estimate.dram_mb = (total_dram / (1024 * 1024)) + self.BASE_DRAM_MB
        
        # Build breakdown
        estimate.breakdown = {
            "base_runtime_vram": self.BASE_VRAM_MB,
            "base_runtime_dram": self.BASE_DRAM_MB,
            "mesh_geometry": round(mesh_vram / (1024 * 1024), 2),
            "textures": round(texture_vram / (1024 * 1024), 2),
            "physics_bodies": round(physics_dram / (1024 * 1024), 2),
            "articulations": round(joint_dram / (1024 * 1024), 2),
            "scene_objects": round((scene_vram + scene_dram) / (1024 * 1024), 2)
        }
        
        return estimate
    
    def _parse_urdf(self, urdf_file: Path) -> Tuple[List[MeshStats], List[TextureStats], int]:
        """
        Parse URDF file to extract mesh and texture references.
        
        Returns:
            Tuple of (mesh_stats_list, texture_stats_list, joint_count)
        """
        meshes: List[MeshStats] = []
        textures: List[TextureStats] = []
        joint_count = 0
        
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        urdf_dir = urdf_file.parent
        
        # Count joints
        for joint in root.findall('.//joint'):
            joint_type = joint.get('type', 'fixed')
            if joint_type != 'fixed':
                joint_count += 1
        
        # Find all mesh references
        seen_meshes = set()
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename', '')
            if not filename or filename in seen_meshes:
                continue
            seen_meshes.add(filename)
            
            # Resolve mesh path
            mesh_path = self._resolve_path(filename, urdf_dir)
            if mesh_path and mesh_path.exists():
                stats = self._analyze_mesh(mesh_path)
                if stats:
                    meshes.append(stats)
                    
                    # Look for associated textures
                    tex_stats = self._find_textures(mesh_path)
                    textures.extend(tex_stats)
        
        return meshes, textures, joint_count
    
    def _resolve_path(self, filename: str, base_dir: Path) -> Optional[Path]:
        """Resolve a filename relative to base directory."""
        # Handle package:// URIs
        if filename.startswith('package://'):
            # Strip package prefix - assume local relative path
            filename = filename.replace('package://', '')
            parts = filename.split('/', 1)
            if len(parts) > 1:
                filename = parts[1]
        
        # Handle relative paths
        if filename.startswith('./') or filename.startswith('../'):
            resolved = (base_dir / filename).resolve()
        elif not os.path.isabs(filename):
            resolved = (base_dir / filename).resolve()
        else:
            resolved = Path(filename)
        
        return resolved if resolved.exists() else None
    
    def _analyze_mesh(self, mesh_path: Path) -> Optional[MeshStats]:
        """Analyze a mesh file to get vertex/triangle counts."""
        suffix = mesh_path.suffix.lower()
        file_size = mesh_path.stat().st_size
        
        stats = MeshStats(
            path=str(mesh_path),
            file_size_bytes=file_size,
            format=suffix[1:] if suffix else "unknown"
        )
        
        try:
            if suffix == '.obj':
                stats.vertices, stats.triangles = self._parse_obj(mesh_path)
            elif suffix == '.stl':
                stats.vertices, stats.triangles = self._parse_stl(mesh_path)
            elif suffix == '.dae':
                stats.vertices, stats.triangles = self._parse_dae(mesh_path)
            else:
                # Estimate from file size
                # Rough heuristic: 100 bytes per vertex on average
                stats.vertices = max(100, file_size // 100)
                stats.triangles = stats.vertices // 3
        except Exception as e:
            logger.warning(f"Error analyzing mesh {mesh_path}: {e}")
            # Estimate from file size
            stats.vertices = max(100, file_size // 100)
            stats.triangles = stats.vertices // 3
        
        return stats
    
    def _parse_obj(self, path: Path) -> Tuple[int, int]:
        """Parse OBJ file to count vertices and faces."""
        vertices = 0
        triangles = 0
        
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if line.startswith('v '):
                    vertices += 1
                elif line.startswith('f '):
                    # Count face vertices
                    parts = line.split()[1:]
                    # Triangulate: n-gon = n-2 triangles
                    triangles += max(1, len(parts) - 2)
        
        return vertices, triangles
    
    def _parse_stl(self, path: Path) -> Tuple[int, int]:
        """Parse STL file (binary or ASCII) to count triangles."""
        with open(path, 'rb') as f:
            header = f.read(80)
            
            # Try binary STL first
            try:
                num_triangles_bytes = f.read(4)
                if len(num_triangles_bytes) == 4:
                    num_triangles = struct.unpack('<I', num_triangles_bytes)[0]
                    # Each triangle in binary STL = 50 bytes
                    # Check if file size matches
                    expected_size = 84 + num_triangles * 50
                    actual_size = path.stat().st_size
                    if abs(expected_size - actual_size) < 100:
                        # Binary STL
                        vertices = num_triangles * 3  # 3 vertices per triangle
                        return vertices, num_triangles
            except:
                pass
        
        # Fallback: ASCII STL
        triangles = 0
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                if 'facet normal' in line.lower():
                    triangles += 1
        
        vertices = triangles * 3
        return vertices, triangles
    
    def _parse_dae(self, path: Path) -> Tuple[int, int]:
        """Parse COLLADA (.dae) file to estimate geometry."""
        vertices = 0
        triangles = 0
        
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            
            # COLLADA namespace
            ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
            
            # Find float_array elements (vertex data)
            for float_array in root.findall('.//c:float_array', ns):
                count = int(float_array.get('count', 0))
                # Assume position data (3 floats per vertex)
                vertices += count // 3
            
            # Find triangles or polylist
            for triangles_elem in root.findall('.//c:triangles', ns):
                count = int(triangles_elem.get('count', 0))
                triangles += count
            
            for polylist in root.findall('.//c:polylist', ns):
                count = int(polylist.get('count', 0))
                triangles += count
            
            # If no namespace match, try without
            if vertices == 0:
                for float_array in root.iter():
                    if 'float_array' in float_array.tag:
                        count = int(float_array.get('count', 0))
                        vertices += count // 3
                
                for elem in root.iter():
                    if 'triangles' in elem.tag:
                        count = int(elem.get('count', 0))
                        triangles += count
                    elif 'polylist' in elem.tag:
                        count = int(elem.get('count', 0))
                        triangles += count
                        
        except Exception as e:
            logger.warning(f"Error parsing DAE {path}: {e}")
            # Estimate from file size
            file_size = path.stat().st_size
            vertices = max(100, file_size // 200)
            triangles = vertices // 3
        
        return max(vertices, 100), max(triangles, 30)
    
    def _find_textures(self, mesh_path: Path) -> List[TextureStats]:
        """Find texture files associated with a mesh."""
        textures: List[TextureStats] = []
        mesh_dir = mesh_path.parent
        
        # Look for common texture patterns
        texture_extensions = ['.png', '.jpg', '.jpeg', '.tga', '.bmp']
        
        # Check for MTL file (OBJ material)
        mtl_path = mesh_path.with_suffix('.mtl')
        if mtl_path.exists():
            textures.extend(self._parse_mtl_textures(mtl_path))
        
        # Look for texture files with similar names
        base_name = mesh_path.stem
        for ext in texture_extensions:
            for pattern in [base_name, f"{base_name}_*", "*_diffuse", "*_color", "*"]:
                for tex_file in mesh_dir.glob(f"{pattern}{ext}"):
                    if tex_file.exists():
                        stats = self._analyze_texture(tex_file)
                        if stats:
                            textures.append(stats)
        
        return textures
    
    def _parse_mtl_textures(self, mtl_path: Path) -> List[TextureStats]:
        """Parse MTL file for texture references."""
        textures: List[TextureStats] = []
        mtl_dir = mtl_path.parent
        
        try:
            with open(mtl_path, 'r', errors='ignore') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('map_') or line.startswith('bump'):
                        parts = line.split()
                        if len(parts) >= 2:
                            tex_filename = parts[-1]
                            tex_path = mtl_dir / tex_filename
                            if tex_path.exists():
                                stats = self._analyze_texture(tex_path)
                                if stats:
                                    textures.append(stats)
        except Exception as e:
            logger.warning(f"Error parsing MTL {mtl_path}: {e}")
        
        return textures
    
    def _analyze_texture(self, tex_path: Path) -> Optional[TextureStats]:
        """Analyze a texture file to get dimensions."""
        file_size = tex_path.stat().st_size
        
        stats = TextureStats(
            path=str(tex_path),
            file_size_bytes=file_size
        )
        
        try:
            # Read image header to get dimensions
            with open(tex_path, 'rb') as f:
                header = f.read(32)
            
            suffix = tex_path.suffix.lower()
            
            if suffix == '.png':
                # PNG: dimensions at bytes 16-23
                if header[0:8] == b'\x89PNG\r\n\x1a\n':
                    stats.width = struct.unpack('>I', header[16:20])[0]
                    stats.height = struct.unpack('>I', header[20:24])[0]
            elif suffix in ['.jpg', '.jpeg']:
                # JPEG: need to find SOF marker
                stats.width, stats.height = self._get_jpeg_dimensions(tex_path)
            else:
                # Estimate from file size (rough heuristic)
                # Assume 3 bytes per pixel uncompressed
                pixels = file_size // 3
                dim = int(pixels ** 0.5)
                stats.width = dim
                stats.height = dim
            
            # Default if dimensions not found
            if stats.width == 0 or stats.height == 0:
                # Estimate 512x512 as default
                stats.width = 512
                stats.height = 512
                
        except Exception as e:
            logger.warning(f"Error analyzing texture {tex_path}: {e}")
            stats.width = 512
            stats.height = 512
        
        return stats
    
    def _get_jpeg_dimensions(self, path: Path) -> Tuple[int, int]:
        """Extract dimensions from JPEG file."""
        try:
            with open(path, 'rb') as f:
                f.seek(0)
                data = f.read()
                
                # Find SOF marker (FFCn where n = 0,1,2,3,5,6,7,9,A,B,D,E,F)
                i = 0
                while i < len(data) - 10:
                    if data[i] == 0xFF:
                        marker = data[i + 1]
                        if marker in [0xC0, 0xC1, 0xC2, 0xC3]:
                            # SOF marker found
                            height = struct.unpack('>H', data[i+5:i+7])[0]
                            width = struct.unpack('>H', data[i+7:i+9])[0]
                            return width, height
                        elif marker == 0xD8:  # SOI
                            i += 2
                        elif marker == 0xD9:  # EOI
                            break
                        elif marker == 0xFF:
                            i += 1
                        else:
                            # Skip to next marker
                            length = struct.unpack('>H', data[i+2:i+4])[0]
                            i += 2 + length
                    else:
                        i += 1
        except Exception:
            pass
        
        return 512, 512  # Default


# Convenience function for quick estimates
def estimate_robot_memory(
    urdf_path: str,
    scene_config: Optional[Dict] = None,
    base_path: Optional[str] = None
) -> Dict[str, Any]:
    """
    Estimate VRAM and DRAM requirements for a robot.
    
    Args:
        urdf_path: Path to URDF file
        scene_config: Optional scene configuration
        base_path: Base path for resolving relative paths
    
    Returns:
        Dict with memory estimates
    """
    estimator = MemoryEstimator(base_path)
    estimate = estimator.estimate_robot_memory(urdf_path, scene_config)
    return estimate.to_dict()
