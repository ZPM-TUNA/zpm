"""
AI Coordinator - Integrates Gemini AI and ElevenLabs for evacuation guidance
Analyzes maze state, robot positions, and provides natural language guidance
"""

import google.generativeai as genai
import os
import json
import requests
from dotenv import load_dotenv
from typing import Dict, List, Optional
from pathfinding import MazeGrid, EvacuationCoordinator
from simulation import run_simulation

# Load environment variables
load_dotenv()


class GeminiAIAnalyzer:
    """Uses Gemini AI to analyze evacuation scenarios and provide guidance"""
    
    def __init__(self):
        genai.configure(api_key=os.getenv('GEMINI_API_KEY'))
        self.model = genai.GenerativeModel('gemini-2.0-flash-exp')
    
    def analyze_evacuation_scenario(
        self,
        maze_state: Dict,
        evacuation_plans: Dict,
        evacuation_status: Dict,
        humans_in_danger: List[str]
    ) -> str:
        """
        Analyze the evacuation scenario and provide guidance
        Robots explore independently, not assigned to specific humans
        """
        prompt = self._build_analysis_prompt(
            maze_state,
            evacuation_plans,
            evacuation_status,
            humans_in_danger
        )
        
        response = self.model.generate_content(prompt)
        return response.text
    
    def _build_analysis_prompt(
        self,
        maze_state: Dict,
        evacuation_plans: Dict,
        evacuation_status: Dict,
        humans_in_danger: List[str]
    ) -> str:
        """Build detailed prompt for Gemini AI"""
        
        prompt = f"""You are an AI evacuation coordinator analyzing a critical rescue scenario in an 8x8 maze.

**MAZE STATE:**
- Grid Size: {maze_state['size']}x{maze_state['size']}
- Obstacles: {len(maze_state['obstacles'])} static obstacles
- Blocked Paths: {len(maze_state['blocked_paths'])} dynamically discovered hazards
- Available Exits: {maze_state['exits']}

**SCOUT ROBOTS (Exploring independently):**
{json.dumps(maze_state['robots'], indent=2)}
Robot Status: Scouts are mapping the area and detecting humans/obstacles dynamically

**HUMANS IN DANGER:**
Total: {len(maze_state['humans'])}
Positions: {json.dumps(maze_state['humans'], indent=2)}
Detected: {evacuation_status['detected_humans']} humans
Critical Priority: {humans_in_danger}

**EVACUATION PLANS:**
{json.dumps(evacuation_plans, indent=2)}

**SYSTEM STATUS:**
- Total Scout Robots: {evacuation_status['total_robots']}
- Total Humans: {evacuation_status['total_humans']}
- Humans with Evacuation Paths: {evacuation_status['humans_with_paths']}
- Total Hazards: {evacuation_status['total_obstacles']}
- Robot Explored Areas: {evacuation_status.get('robot_explored_areas', {})}

**YOUR TASK:**
Provide clear, calm evacuation instructions for the people in the maze that:
1. Tells each person their evacuation route (position → exit)
2. Warns about obstacles/hazards to avoid
3. Identifies which people are in most danger
4. Provides reassurance that scouts are mapping safe routes
5. Gives simple directional guidance (north/south/east/west)

Keep the message concise (2-4 sentences per person), professional, and reassuring. This will be spoken aloud to people in an emergency situation.
"""
        
        return prompt
    
    def analyze_path_blockage(
        self,
        robot_id: str,
        blocked_position: tuple,
        new_path: List[tuple],
        maze_state: Dict
    ) -> str:
        """Analyze path blockage and provide rerouting guidance"""
        
        prompt = f"""A path has been blocked during an evacuation rescue operation.

**SITUATION:**
- Robot: {robot_id}
- Blocked Position: {blocked_position}
- New Path Length: {len(new_path)} steps
- Maze Size: {maze_state['size']}x{maze_state['size']}

**NEW ROUTE:**
{new_path}

Provide a brief, calm update message (1-2 sentences) explaining:
1. That a path has been blocked
2. That a new route has been calculated
3. Reassurance that the rescue is still on track

Keep it professional and reassuring for emergency broadcast.
"""
        
        response = self.model.generate_content(prompt)
        return response.text
    
    def prioritize_rescues(
        self,
        humans: Dict[str, tuple],
        evacuation_plans: Dict,
        maze_state: Dict
    ) -> List[str]:
        """Use AI to prioritize which humans are in most danger based on evacuation paths"""
        
        prompt = f"""Analyze this evacuation scenario and identify humans in most critical danger.

**HUMANS:**
{json.dumps(humans, indent=2)}

**EVACUATION PLANS:**
{json.dumps(evacuation_plans, indent=2)}

**MAZE HAZARDS:**
- Obstacles: {maze_state['obstacles']}
- Blocked Paths: {maze_state['blocked_paths']}
- Exits: {maze_state['exits']}

Based on:
1. Distance from exits (longer = more danger)
2. Proximity to hazards/obstacles
3. Path availability (no path = critical)
4. Path complexity through dangerous areas

Return ONLY a JSON array of human IDs in priority order (most critical first).
Example: ["human_2", "human_1", "human_3"]
"""
        
        response = self.model.generate_content(prompt)
        try:
            # Extract JSON from response
            text = response.text.strip()
            if text.startswith('```json'):
                text = text[7:]
            if text.endswith('```'):
                text = text[:-3]
            return json.loads(text.strip())
        except:
            # Fallback to all humans
            return list(humans.keys())


class ElevenLabsVoice:
    """Converts text to natural speech using ElevenLabs API"""
    
    def __init__(self):
        self.api_key = os.getenv('ELEVENLABS_API_KEY')
        self.voice_id = os.getenv('ELEVENLABS_VOICE_ID', 'EXAVITQu4vr4xnSDxMaL')  # Default: Sarah
        self.api_url = f"https://api.elevenlabs.io/v1/text-to-speech/{self.voice_id}"
    
    def text_to_speech(self, text: str, output_file: str = "evacuation_guidance.mp3") -> bool:
        """
        Convert text to speech and save to file
        Returns True if successful
        """
        if not self.api_key:
            print("Warning: ELEVENLABS_API_KEY not set. Skipping voice generation.")
            return False
        
        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.api_key
        }
        
        data = {
            "text": text,
            "model_id": "eleven_monolingual_v1",
            "voice_settings": {
                "stability": 0.5,
                "similarity_boost": 0.75,
                "style": 0.0,
                "use_speaker_boost": True
            }
        }
        
        try:
            response = requests.post(self.api_url, json=data, headers=headers)
            
            if response.status_code == 200:
                with open(output_file, 'wb') as f:
                    f.write(response.content)
                print(f"✓ Voice guidance saved to {output_file}")
                return True
            else:
                print(f"✗ ElevenLabs API error: {response.status_code} - {response.text}")
                return False
        except Exception as e:
            print(f"✗ Error generating voice: {e}")
            return False
    
    def stream_text_to_speech(self, text: str) -> Optional[bytes]:
        """
        Convert text to speech and return audio bytes for streaming
        """
        if not self.api_key:
            return None
        
        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.api_key
        }
        
        data = {
            "text": text,
            "model_id": "eleven_monolingual_v1",
            "voice_settings": {
                "stability": 0.5,
                "similarity_boost": 0.75
            }
        }
        
        try:
            response = requests.post(self.api_url, json=data, headers=headers)
            if response.status_code == 200:
                return response.content
        except Exception as e:
            print(f"Error streaming voice: {e}")
        
        return None


class EvacuationAICoordinator:
    """Main coordinator integrating pathfinding, AI analysis, and voice guidance"""
    
    def __init__(self, maze: MazeGrid):
        self.maze = maze
        self.evacuation_coordinator = EvacuationCoordinator(maze)
        self.ai_analyzer = GeminiAIAnalyzer()
        self.voice_generator = ElevenLabsVoice()
        self.current_guidance = None
        self.voice_file = None
    
    def analyze_and_guide(self, generate_voice: bool = True) -> Dict:
        """
        Complete analysis and guidance generation
        Calculates evacuation paths for all humans (robots explore independently)
        Returns dict with guidance text, voice file, and evacuation plans
        """
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
        
        # Generate AI guidance
        guidance_text = self.ai_analyzer.analyze_evacuation_scenario(
            maze_state,
            evacuation_plans,
            evacuation_status,
            humans_in_danger
        )
        
        self.current_guidance = guidance_text
        
        result = {
            'guidance_text': guidance_text,
            'humans_in_danger': humans_in_danger,
            'evacuation_plans': evacuation_plans,
            'evacuation_status': evacuation_status,
            'maze_state': maze_state,
            'voice_file': None
        }
        
        # Generate voice guidance
        if generate_voice:
            voice_file = "evacuation_guidance.mp3"
            if self.voice_generator.text_to_speech(guidance_text, voice_file):
                result['voice_file'] = voice_file
                self.voice_file = voice_file
        
        return result
    
    def handle_path_blocked(self, robot_id: str, blocked_pos: tuple, generate_voice: bool = True) -> Dict:
        """
        Handle blocked path scenario - robot detected obstacle
        Recalculates evacuation paths for all affected humans
        """
        # Robot detected obstacle - recalculate all affected paths
        result_data = self.evacuation_coordinator.robot_detected_obstacle(robot_id, blocked_pos)
        
        maze_state = self.maze.get_state()
        
        # Build summary of affected paths
        affected_summary = []
        for human_id, changes in result_data['affected_humans'].items():
            if changes.get('path_changed'):
                affected_summary.append(f"{human_id}: new path {changes.get('new_distance', 'N/A')} steps")
        
        # Generate AI guidance for blockage
        guidance_text = self.ai_analyzer.analyze_path_blockage(
            robot_id,
            blocked_pos,
            affected_summary,
            maze_state
        )
        
        result = {
            'success': True,
            'guidance_text': guidance_text,
            'robot_id': robot_id,
            'blocked_position': blocked_pos,
            'affected_humans': result_data['affected_humans'],
            'total_obstacles': result_data['total_obstacles'],
            'voice_file': None
        }
        
        # Generate voice
        if generate_voice:
            voice_file = f"blockage_update_{robot_id}.mp3"
            if self.voice_generator.text_to_speech(guidance_text, voice_file):
                result['voice_file'] = voice_file
        
        return result
    
    def get_flutter_update(self) -> Dict:
        """
        Get formatted data for Flutter mobile app
        Robots explore independently, humans have evacuation paths
        """
        evacuation_status = self.evacuation_coordinator.get_evacuation_status()
        maze_state = self.maze.get_state()
        
        return {
            'timestamp': None,  # Add timestamp in production
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
                'voice_file': self.voice_file
            },
            'system_status': {
                'robots_exploring': len(maze_state['robots']),
                'humans_detected': len(self.evacuation_coordinator.detected_humans),
                'total_humans': len(maze_state['humans']),
                'humans_with_paths': len(self.evacuation_coordinator.human_evacuation_paths)
            }
        }


if __name__ == "__main__":
    import sys
    if "--simulate" in sys.argv:
        run_simulation(num_robots=3)
    else:
        print("Run with --simulate to start the building simulation.")
        print("=" * 60)
        print("TESTING AI EVACUATION COORDINATOR")
        print("(Dynamic Pathfinding - Robots Explore Independently)")
        print("=" * 60)
        
        # Create maze
        maze = MazeGrid(8)
        
        # Add obstacles
        maze.add_obstacle(2, 2)
        maze.add_obstacle(2, 3)
        maze.add_obstacle(5, 5)
        
        # Add exits
        maze.add_exit(7, 7)
        maze.add_exit(0, 7)
        
        # Add scout robots (they explore independently)
        maze.add_robot("scout_1", 0, 0)
        maze.add_robot("scout_2", 7, 0)
        
        # Add humans
        maze.add_human("human_1", 4, 4)
        maze.add_human("human_2", 5, 3)
        maze.add_human("human_3", 6, 6)
        
        # Create AI coordinator
        ai_coordinator = EvacuationAICoordinator(maze)
        
        # Simulate robot exploration
        print("\n1. Robots Exploring...")
        ai_coordinator.evacuation_coordinator.update_robot_exploration("scout_1", (1, 1))
        ai_coordinator.evacuation_coordinator.update_robot_exploration("scout_1", (2, 1))
        ai_coordinator.evacuation_coordinator.robot_detected_human("scout_1", "human_1", (4, 4))
        print("✓ Scout 1 detected human_1")
        
        # Analyze and generate guidance
        print("\n2. Generating AI Evacuation Guidance...")
        result = ai_coordinator.analyze_and_guide(generate_voice=False)
        
        print("\n" + "=" * 60)
        print("EVACUATION GUIDANCE:")
        print("=" * 60)
        print(result['guidance_text'])
        print("\n" + "=" * 60)
        
        print(f"\nHumans in danger (priority order): {result['humans_in_danger']}")
        print(f"\nEvacuation Plans:")
        for human_id, plan in result['evacuation_plans'].items():
            print(f"  {human_id}: {plan.get('distance_to_exit', 'N/A')} steps to exit {plan.get('exit_position', 'N/A')}")
        
        if result['voice_file']:
            print(f"\n✓ Voice guidance generated: {result['voice_file']}")
        
        # Test dynamic obstacle detection
        print("\n\n" + "=" * 60)
        print("3. SIMULATING DYNAMIC OBSTACLE DETECTION...")
        print("=" * 60)
        print("Scout 2 detects obstacle at (4, 5)")
        blockage_result = ai_coordinator.handle_path_blocked("scout_2", (4, 5), generate_voice=False)
        
        if blockage_result['success']:
            print(f"\n✓ Obstacle added at {blockage_result['blocked_position']}")
            print(f"✓ Total obstacles: {blockage_result['total_obstacles']}")
            print(f"\nAffected Humans:")
            for human_id, changes in blockage_result['affected_humans'].items():
                if changes.get('path_changed'):
                    print(f"  {human_id}: Path recalculated ({changes.get('new_distance', 'N/A')} steps)")
            
            print(f"\nAI Update:")
            print(blockage_result['guidance_text'])
        
        # Get Flutter update
        print("\n\n" + "=" * 60)
        print("4. FLUTTER APP DATA:")
        print("=" * 60)
        flutter_data = ai_coordinator.get_flutter_update()
        print(f"Robots exploring: {flutter_data['system_status']['robots_exploring']}")
        print(f"Humans detected: {flutter_data['system_status']['humans_detected']}")
        print(f"Humans with paths: {flutter_data['system_status']['humans_with_paths']}")
        print(f"\nFull data:")
        print(json.dumps(flutter_data, indent=2))
        
        print("\n" + "=" * 60)
        print("✓ Test Complete!")
        print("=" * 60)

