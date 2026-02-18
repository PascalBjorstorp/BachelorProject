"""
Map Utilities for GPU AMCL

Handles loading and processing of occupancy grid maps.
Provides utilities for coordinate transforms between world and map frames.
"""

from typing import Tuple, Optional
import numpy as np
import yaml
import os

# Try to import PIL for image loading
try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


class MapProcessor:
    """
    Occupancy Grid Map Handler.
    
    Loads maps from ROS map_server format (YAML + image) and provides
    coordinate transform utilities.
    
    Map format:
        - YAML file with resolution, origin, image path
        - Image file with occupancy values:
            - White (255): Free space
            - Black (0): Occupied
            - Gray (205): Unknown
    
    Example:
        >>> processor = MapProcessor()
        >>> map_data, resolution, origin = processor.load_map('map.yaml')
    """
    
    def __init__(self):
        """Initialize map processor."""
        self.map_data: Optional[np.ndarray] = None
        self.resolution: float = 0.05
        self.origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.map_loaded = False
    
    def load_map(self, yaml_path: str) -> Tuple[np.ndarray, float, Tuple[float, float, float]]:
        """
        Load map from YAML configuration file.
        
        Args:
            yaml_path: Path to map YAML file (e.g., 'map.yaml')
        
        Returns:
            Tuple of:
                - map_data: 2D numpy array (0=free, 100=occupied, -1=unknown)
                - resolution: meters per pixel
                - origin: (x, y, theta) of map origin
        """
        # Load YAML config
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.resolution = float(config['resolution'])
        self.origin = tuple(config['origin'])  # [x, y, theta]
        
        # Load image
        image_path = config['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        
        # Load and process image
        if PIL_AVAILABLE:
            img = Image.open(image_path)
            img_array = np.array(img, dtype=np.uint8)
        else:
            # Fallback: try OpenCV
            import cv2
            img_array = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if img_array is None:
                raise FileNotFoundError(f"Could not load map image: {image_path}")
        
        # Handle multi-channel images (take first channel)
        if len(img_array.shape) > 2:
            img_array = img_array[:, :, 0]
        
        # Convert to occupancy grid format
        # ROS convention: 0=occupied, 100=free in OccupancyGrid
        # Image convention: 0=black=occupied, 255=white=free
        self.map_data = self._image_to_occupancy(
            img_array,
            config.get('negate', 0),
            config.get('occupied_thresh', 0.65),
            config.get('free_thresh', 0.196)
        )
        
        self.map_loaded = True
        print(f"[MapProcessor] Loaded map: {self.map_data.shape}, res={self.resolution}m")
        
        return self.map_data, self.resolution, self.origin
    
    def _image_to_occupancy(
        self,
        img: np.ndarray,
        negate: int,
        occupied_thresh: float,
        free_thresh: float
    ) -> np.ndarray:
        """
        Convert image to occupancy grid.
        
        Args:
            img: Grayscale image array (0-255)
            negate: If 1, invert black/white interpretation
            occupied_thresh: Threshold for occupied cells
            free_thresh: Threshold for free cells
        
        Returns:
            Occupancy grid: 0=free, 100=occupied, -1=unknown
        """
        # Normalize to [0, 1]
        normalized = img.astype(np.float32) / 255.0
        
        # Optionally negate
        if negate:
            normalized = 1.0 - normalized
        
        # Create occupancy grid
        occupancy = np.full(img.shape, -1, dtype=np.int8)  # Default: unknown
        occupancy[normalized > (1.0 - free_thresh)] = 0     # Free (white in image)
        occupancy[normalized < (1.0 - occupied_thresh)] = 100  # Occupied (black in image)
        
        # Flip vertically (image origin is top-left, map origin is bottom-left)
        occupancy = np.flipud(occupancy)
        
        return occupancy
    
    def load_from_nav_msgs(self, occupancy_grid_msg) -> None:
        """
        Load map from nav_msgs/OccupancyGrid message.
        
        Args:
            occupancy_grid_msg: nav_msgs/OccupancyGrid message
        """
        info = occupancy_grid_msg.info
        
        self.resolution = info.resolution
        self.origin = (
            info.origin.position.x,
            info.origin.position.y,
            0.0  # Assuming no rotation
        )
        
        # Reshape data to 2D
        width = info.width
        height = info.height
        self.map_data = np.array(occupancy_grid_msg.data, dtype=np.int8).reshape((height, width))
        
        self.map_loaded = True
        print(f"[MapProcessor] Loaded map from OccupancyGrid: {width}x{height}")
    
    def world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        """
        Convert world coordinates to map pixel coordinates.
        
        Args:
            wx: World X coordinate (meters)
            wy: World Y coordinate (meters)
        
        Returns:
            Tuple of (mx, my) map pixel coordinates
        """
        ox, oy, _ = self.origin
        mx = int((wx - ox) / self.resolution)
        my = int((wy - oy) / self.resolution)
        return mx, my
    
    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """
        Convert map pixel coordinates to world coordinates.
        
        Args:
            mx: Map X coordinate (pixels)
            my: Map Y coordinate (pixels)
        
        Returns:
            Tuple of (wx, wy) world coordinates (meters)
        """
        ox, oy, _ = self.origin
        wx = ox + (mx + 0.5) * self.resolution
        wy = oy + (my + 0.5) * self.resolution
        return wx, wy
    
    def is_in_bounds(self, mx: int, my: int) -> bool:
        """Check if map coordinates are within bounds."""
        if self.map_data is None:
            return False
        height, width = self.map_data.shape
        return 0 <= mx < width and 0 <= my < height
    
    def is_free(self, wx: float, wy: float) -> bool:
        """Check if world position is in free space."""
        mx, my = self.world_to_map(wx, wy)
        if not self.is_in_bounds(mx, my):
            return False
        return self.map_data[my, mx] == 0
    
    def is_occupied(self, wx: float, wy: float) -> bool:
        """Check if world position is occupied."""
        mx, my = self.world_to_map(wx, wy)
        if not self.is_in_bounds(mx, my):
            return True  # Outside map = obstacle
        return self.map_data[my, mx] == 100
    
    def get_free_cells(self) -> np.ndarray:
        """
        Get array of all free cell coordinates.
        
        Returns:
            (M, 2) array of [x, y] world coordinates for free cells
        """
        if self.map_data is None:
            return np.array([])
        
        free_coords = np.argwhere(self.map_data == 0)
        world_coords = np.zeros((len(free_coords), 2), dtype=np.float32)
        
        for i, (my, mx) in enumerate(free_coords):
            wx, wy = self.map_to_world(mx, my)
            world_coords[i] = [wx, wy]
        
        return world_coords
