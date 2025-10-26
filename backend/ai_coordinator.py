"""
AI Coordinator - Simplified for ZeroPanic
Provides basic evacuation guidance without external API dependencies
"""

import json
from typing import Dict, List, Optional
from pathfinding import MazeGrid, EvacuationCoordinator


class SimpleEvacuationAnalyzer:
    """Simple evacuation analyzer without external AI dependencies"""
    
    def analyze_evacuation_scenario(
        self,
        maze_state: Dict,
        evacuation_plans: Dict,
        evacuation_status: Dict,
        humans_in_danger: List[str]
    ) -> str:
        """Generate simple evacuation guidance"""
        
        total_humans = evacuation_status['total_humans']
        detected_humans = evacuation_status['detected_humans']
        humans_with_paths = evacuation_status['humans_with_paths']
        
        guidance = f"Evacuation Status: {detected_humans}/{total_humans} humans detected. "
        
        if humans_with_paths > 0:
            guidance += f"{humans_with_paths} evacuation paths calculated. "
        
        if evacuation_plans:
            guidance += "Follow the marked evacuation routes to the nearest exits. "
            guidance += "Avoid obstacles and stay calm. "
        else:
            guidance += "Robots are still mapping the area. Stay in place until evacuation routes are available. "
        
        return guidance
    
    def prioritize_rescues(
        self,
        humans: Dict[str, tuple],
        evacuation_plans: Dict,
        maze_state: Dict
    ) -> List[str]:
        """Simple priority based on evacuation path availability"""
        priority_order = []
        
        # Humans without paths are highest priority
        for human_id in humans.keys():
            if human_id not in evacuation_plans or not evacuation_plans[human_id].get('evacuation_path'):
                priority_order.append(human_id)
        
        # Then humans with longer paths
        humans_with_paths = [(human_id, plan.get('distance_to_exit', 0)) 
                           for human_id, plan in evacuation_plans.items() 
                           if plan.get('evacuation_path')]
        humans_with_paths.sort(key=lambda x: x[1], reverse=True)
        
        for human_id, _ in humans_with_paths:
            if human_id not in priority_order:
                priority_order.append(human_id)
        
        return priority_order


class EvacuationAICoordinator:
    """Simplified coordinator for evacuation guidance"""
    
    def __init__(self, maze: MazeGrid):
        self.maze = maze
        self.evacuation_coordinator = EvacuationCoordinator(maze)
        self.ai_analyzer = SimpleEvacuationAnalyzer()
        self.current_guidance = None
    
    def analyze_and_guide(self, generate_voice: bool = False) -> Dict:
        """Generate evacuation guidance"""
        # Calculate evacuation paths for all humans
        evacuation_plans = self.evacuation_coordinator.calculate_evacuation_paths()
        
        # Get maze state
        maze_state = self.maze.get_state()
        
        # Get evacuation status
        evacuation_status = self.evacuation_coordinator.get_evacuation_status()
        
        # Prioritize humans in danger
        humans_in_danger = self.ai_analyzer.prioritize_rescues(
            self.maze.humans,
            evacuation_plans,
            maze_state
        )
        
        # Generate guidance
        guidance_text = self.ai_analyzer.analyze_evacuation_scenario(
            maze_state,
            evacuation_plans,
            evacuation_status,
            humans_in_danger
        )
        
        self.current_guidance = guidance_text
        
        return {
            'guidance_text': guidance_text,
            'humans_in_danger': humans_in_danger,
            'evacuation_plans': evacuation_plans,
            'evacuation_status': evacuation_status,
            'maze_state': maze_state,
            'voice_file': None  # Voice generation disabled
        }
    
    def handle_path_blocked(self, robot_id: str, blocked_pos: tuple, generate_voice: bool = False) -> Dict:
        """Handle blocked path scenario"""
        # Robot detected obstacle - recalculate all affected paths
        result_data = self.evacuation_coordinator.robot_detected_obstacle(robot_id, blocked_pos)
        
        maze_state = self.maze.get_state()
        
        # Generate simple guidance for blockage
        guidance_text = f"Path blocked at {blocked_pos}. New evacuation routes are being calculated. Stay calm and follow updated directions."
        
        return {
            'success': True,
            'guidance_text': guidance_text,
            'robot_id': robot_id,
            'blocked_position': blocked_pos,
            'affected_humans': result_data['affected_humans'],
            'total_obstacles': result_data['total_obstacles'],
            'voice_file': None
        }
    
    def get_flutter_update(self) -> Dict:
        """Get formatted data for Flutter mobile app"""
        evacuation_status = self.evacuation_coordinator.get_evacuation_status()
        maze_state = self.maze.get_state()
        
        return {
            'timestamp': None,
            'maze': {
                'size': maze_state['size'],
                'obstacles': maze_state['obstacles'],
                'blocked_paths': maze_state['blocked_paths'],
                'exits': maze_state['exits']
            },
            'robots': [
                {
                    'id': robot_id,
                    'position': pos,
                    'explored_area': self.evacuation_coordinator.robot_explored_areas.get(robot_id, []),
                    'status': 'exploring',
                    'cells_explored': len(self.evacuation_coordinator.robot_explored_areas.get(robot_id, []))
                }
                for robot_id, pos in maze_state['robots'].items()
            ],
            'humans': [
                {
                    'id': human_id,
                    'position': pos,
                    'evacuation_path': self.evacuation_coordinator.human_evacuation_paths.get(human_id, {}).get('path', []),
                    'exit_target': self.evacuation_coordinator.human_evacuation_paths.get(human_id, {}).get('exit'),
                    'distance_to_exit': self.evacuation_coordinator.human_evacuation_paths.get(human_id, {}).get('distance'),
                    'detected': human_id in self.evacuation_coordinator.detected_humans,
                    'status': 'in_danger'
                }
                for human_id, pos in maze_state['humans'].items()
            ],
            'guidance': {
                'text': self.current_guidance,
                'voice_file': None
            },
            'system_status': {
                'robots_exploring': len(maze_state['robots']),
                'humans_detected': len(self.evacuation_coordinator.detected_humans),
                'total_humans': len(maze_state['humans']),
                'humans_with_paths': len(self.evacuation_coordinator.human_evacuation_paths)
            }
        }