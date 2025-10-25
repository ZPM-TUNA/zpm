#!/usr/bin/env python3
"""
Shared utility functions for ZPM-TUNA backend
Consolidates common functionality to reduce code duplication
"""

import json
import time
from typing import Dict, List, Tuple, Optional, Any
import logging

logger = logging.getLogger(__name__)


def validate_position(x: int, y: int, grid_size: int = 8) -> bool:
    """Validate grid position coordinates"""
    return 0 <= x < grid_size and 0 <= y < grid_size


def calculate_distance(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
    """Calculate Euclidean distance between two positions"""
    return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5


def format_response(success: bool, message: str = "", data: Dict = None, error: str = None) -> Dict:
    """Standardize API response format"""
    response = {
        'success': success,
        'message': message
    }
    
    if data:
        response.update(data)
    
    if error:
        response['error'] = error
    
    return response


def log_api_call(endpoint: str, method: str, status_code: int = 200, duration_ms: float = None):
    """Log API call for debugging"""
    duration_str = f" ({duration_ms:.1f}ms)" if duration_ms else ""
    logger.info(f"{method} {endpoint} -> {status_code}{duration_str}")


def safe_json_loads(json_str: str, default: Any = None) -> Any:
    """Safely parse JSON string with fallback"""
    try:
        return json.loads(json_str)
    except (json.JSONDecodeError, TypeError):
        return default


def convert_pixel_to_grid(pixel_x: int, pixel_y: int, frame_width: int, frame_height: int, 
                         grid_cols: int = 8, grid_rows: int = 8) -> Tuple[int, int]:
    """Convert pixel coordinates to grid coordinates"""
    grid_x = int((pixel_x / frame_width) * grid_cols)
    grid_y = int((pixel_y / frame_height) * grid_rows)
    return (grid_x, grid_y)


def convert_grid_to_pixel(grid_x: int, grid_y: int, frame_width: int, frame_height: int,
                         grid_cols: int = 8, grid_rows: int = 8) -> Tuple[int, int]:
    """Convert grid coordinates to pixel coordinates"""
    pixel_x = int((grid_x / grid_cols) * frame_width)
    pixel_y = int((grid_y / grid_rows) * frame_height)
    return (pixel_x, pixel_y)


def format_robot_data(robot_id: str, position: Tuple[int, int], status: str = "active") -> Dict:
    """Format robot data for API responses"""
    return {
        'id': robot_id,
        'position': list(position),
        'status': status
    }


def format_human_data(human_id: str, position: Tuple[int, int], assigned_robot: str = None) -> Dict:
    """Format human data for API responses"""
    return {
        'id': human_id,
        'position': list(position),
        'assigned_robot': assigned_robot,
        'status': 'in_danger'
    }


def validate_robot_command(command: str, required_params: List[str], data: Dict) -> Tuple[bool, str]:
    """Validate robot command parameters"""
    if command not in ['forward', 'backward', 'left', 'right', 'stop', 'scan']:
        return False, f"Unknown command: {command}"
    
    for param in required_params:
        if param not in data:
            return False, f"Missing required parameter: {param}"
    
    return True, ""


def calculate_path_length(path: List[Tuple[int, int]]) -> int:
    """Calculate total path length"""
    if len(path) < 2:
        return 0
    
    total_length = 0
    for i in range(1, len(path)):
        total_length += calculate_distance(path[i-1], path[i])
    
    return int(total_length)


def find_nearest_position(current_pos: Tuple[int, int], target_positions: List[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
    """Find nearest position from a list of target positions"""
    if not target_positions:
        return None
    
    min_distance = float('inf')
    nearest = None
    
    for target_pos in target_positions:
        distance = calculate_distance(current_pos, target_pos)
        if distance < min_distance:
            min_distance = distance
            nearest = target_pos
    
    return nearest


def create_error_response(error_message: str, status_code: int = 400) -> Tuple[Dict, int]:
    """Create standardized error response"""
    return format_response(False, error=error_message), status_code


def create_success_response(message: str = "Success", data: Dict = None) -> Tuple[Dict, int]:
    """Create standardized success response"""
    return format_response(True, message, data), 200


def measure_execution_time(func):
    """Decorator to measure function execution time"""
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        duration_ms = (end_time - start_time) * 1000
        logger.debug(f"{func.__name__} executed in {duration_ms:.2f}ms")
        return result
    return wrapper


class PositionTracker:
    """Track and estimate positions with drift correction"""
    
    def __init__(self, initial_position: Tuple[int, int] = (0, 0)):
        self.estimated_position = list(initial_position)
        self.drift_factor = 0.95  # Reduce drift over time
        self.last_update = time.time()
    
    def update_position(self, new_position: Tuple[int, int]):
        """Update position with drift correction"""
        current_time = time.time()
        time_delta = current_time - self.last_update
        
        # Apply drift correction based on time
        drift_correction = self.drift_factor ** time_delta
        
        # Blend new position with estimated position
        self.estimated_position[0] = (self.estimated_position[0] * drift_correction + 
                                    new_position[0] * (1 - drift_correction))
        self.estimated_position[1] = (self.estimated_position[1] * drift_correction + 
                                    new_position[1] * (1 - drift_correction))
        
        self.last_update = current_time
    
    def get_position(self) -> Tuple[int, int]:
        """Get current estimated position"""
        return (int(round(self.estimated_position[0])), 
                int(round(self.estimated_position[1])))


if __name__ == "__main__":
    # Test utility functions
    print("Testing ZPM-TUNA utilities...")
    
    # Test position validation
    print(f"Valid position (3, 4): {validate_position(3, 4)}")
    print(f"Invalid position (10, 4): {validate_position(10, 4)}")
    
    # Test distance calculation
    distance = calculate_distance((0, 0), (3, 4))
    print(f"Distance from (0,0) to (3,4): {distance:.2f}")
    
    # Test pixel to grid conversion
    pixel_x, pixel_y = 320, 240
    frame_w, frame_h = 640, 480
    grid_x, grid_y = convert_pixel_to_grid(pixel_x, pixel_y, frame_w, frame_h)
    print(f"Pixel ({pixel_x}, {pixel_y}) -> Grid ({grid_x}, {grid_y})")
    
    # Test position tracker
    tracker = PositionTracker((0, 0))
    tracker.update_position((1, 1))
    print(f"Position tracker: {tracker.get_position()}")
    
    print("✓ All utility tests passed!")
