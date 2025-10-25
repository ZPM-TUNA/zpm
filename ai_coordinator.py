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
        robot_assignments: Dict,
        evacuation_status: Dict,
        humans_in_danger: List[str]
    ) -> str:
        """
        Analyze the evacuation scenario and provide guidance
        """
        prompt = self._build_analysis_prompt(
            maze_state,
            robot_assignments,
            evacuation_status,
            humans_in_danger
        )
        
        response = self.model.generate_content(prompt)
        return response.text
    
    def _build_analysis_prompt(
        self,
        maze_state: Dict,
        robot_assignments: Dict,
        evacuation_status: Dict,
        humans_in_danger: List[str]
    ) -> str:
        """Build detailed prompt for Gemini AI"""
        
        prompt = f"""You are an AI evacuation coordinator analyzing a critical rescue scenario in an 8x8 maze.

**MAZE STATE:**
- Grid Size: {maze_state['size']}x{maze_state['size']}
- Obstacles: {len(maze_state['obstacles'])} static obstacles
- Blocked Paths: {len(maze_state['blocked_paths'])} dynamically blocked paths
- Available Exits: {maze_state['exits']}

**ROBOTS:**
{json.dumps(maze_state['robots'], indent=2)}

**HUMANS IN DANGER:**
Total: {len(maze_state['humans'])}
Positions: {json.dumps(maze_state['humans'], indent=2)}
Critical: {humans_in_danger}

**CURRENT ASSIGNMENTS:**
{json.dumps(robot_assignments, indent=2)}

**EVACUATION STATUS:**
- Total Robots: {evacuation_status['total_robots']}
- Total Humans: {evacuation_status['total_humans']}
- Assigned Humans: {evacuation_status['assigned_humans']}
- Unassigned Humans: {evacuation_status['total_humans'] - evacuation_status['assigned_humans']}

**YOUR TASK:**
Provide a clear, calm, and actionable evacuation guidance message that:
1. Summarizes the current situation
2. Identifies who is in most danger
3. Explains the rescue plan for each robot
4. Provides reassurance and clear next steps
5. Warns about any blocked paths or hazards

Keep the message concise (2-3 sentences), professional, and reassuring. This will be spoken aloud to people in an emergency situation.
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
        robot_assignments: Dict,
        maze_state: Dict
    ) -> List[str]:
        """Use AI to prioritize which humans are in most danger"""
        
        prompt = f"""Analyze this evacuation scenario and identify humans in most critical danger.

**HUMANS:**
{json.dumps(humans, indent=2)}

**ROBOT ASSIGNMENTS:**
{json.dumps(robot_assignments, indent=2)}

**MAZE HAZARDS:**
- Obstacles: {maze_state['obstacles']}
- Blocked Paths: {maze_state['blocked_paths']}
- Exits: {maze_state['exits']}

Based on:
1. Distance from exits
2. Proximity to hazards
3. Whether a robot is assigned
4. Path complexity

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
        Returns dict with guidance text, voice file, and evacuation plan
        """
        # Get evacuation assignments
        assignments = self.evacuation_coordinator.assign_rescue_paths()
        
        # Get maze state
        maze_state = self.maze.get_state()
        
        # Get evacuation status
        evacuation_status = self.evacuation_coordinator.get_evacuation_status()
        
        # Prioritize humans in danger
        humans_in_danger = self.ai_analyzer.prioritize_rescues(
            self.maze.humans,
            assignments,
            maze_state
        )
        
        # Generate AI guidance
        guidance_text = self.ai_analyzer.analyze_evacuation_scenario(
            maze_state,
            assignments,
            evacuation_status,
            humans_in_danger
        )
        
        self.current_guidance = guidance_text
        
        result = {
            'guidance_text': guidance_text,
            'humans_in_danger': humans_in_danger,
            'robot_assignments': assignments,
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
        Handle blocked path scenario
        """
        # Update maze and recalculate path
        success = self.evacuation_coordinator.handle_blocked_path(robot_id, blocked_pos)
        
        if not success:
            return {
                'success': False,
                'message': 'Could not find alternative path'
            }
        
        # Get new path
        new_path = self.evacuation_coordinator.robot_paths.get(robot_id, [])
        maze_state = self.maze.get_state()
        
        # Generate AI guidance for blockage
        guidance_text = self.ai_analyzer.analyze_path_blockage(
            robot_id,
            blocked_pos,
            new_path,
            maze_state
        )
        
        result = {
            'success': True,
            'guidance_text': guidance_text,
            'new_path': new_path,
            'robot_id': robot_id,
            'blocked_position': blocked_pos,
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
                    'path': evacuation_status['robot_paths'].get(robot_id, {}).get('path', []),
                    'status': 'active'
                }
                for robot_id, pos in maze_state['robots'].items()
            ],
            'humans': [
                {
                    'id': human_id,
                    'position': pos,
                    'assigned_robot': next(
                        (rid for hid, rid in self.evacuation_coordinator.human_assignments.items() if hid == human_id),
                        None
                    ),
                    'status': 'in_danger'
                }
                for human_id, pos in maze_state['humans'].items()
            ],
            'guidance': {
                'text': self.current_guidance,
                'voice_file': self.voice_file
            }
        }


if __name__ == "__main__":
    # Test the AI coordinator
    print("Testing AI Evacuation Coordinator...\n")
    
    # Create maze
    maze = MazeGrid(8)
    
    # Add obstacles
    maze.add_obstacle(2, 2)
    maze.add_obstacle(2, 3)
    maze.add_obstacle(5, 5)
    
    # Add exits
    maze.add_exit(7, 7)
    maze.add_exit(0, 7)
    
    # Add robots
    maze.add_robot("robot_1", 0, 0)
    maze.add_robot("robot_2", 7, 0)
    
    # Add humans
    maze.add_human("human_1", 4, 4)
    maze.add_human("human_2", 5, 3)
    maze.add_human("human_3", 6, 6)
    
    # Create AI coordinator
    ai_coordinator = EvacuationAICoordinator(maze)
    
    # Analyze and generate guidance
    print("Generating evacuation guidance...\n")
    result = ai_coordinator.analyze_and_guide(generate_voice=True)
    
    print("=" * 60)
    print("EVACUATION GUIDANCE:")
    print("=" * 60)
    print(result['guidance_text'])
    print("\n" + "=" * 60)
    
    print(f"\nHumans in danger (priority order): {result['humans_in_danger']}")
    print(f"\nRobot Assignments:")
    for robot_id, assignment in result['robot_assignments'].items():
        print(f"  {robot_id} → {assignment['target_human']} (distance: {assignment['total_distance']})")
    
    if result['voice_file']:
        print(f"\n✓ Voice guidance generated: {result['voice_file']}")
    
    # Test path blockage
    print("\n\n" + "=" * 60)
    print("SIMULATING PATH BLOCKAGE...")
    print("=" * 60)
    blockage_result = ai_coordinator.handle_path_blocked("robot_1", (1, 1), generate_voice=True)
    
    if blockage_result['success']:
        print(f"\nBlockage Update:")
        print(blockage_result['guidance_text'])
        print(f"\nNew path length: {len(blockage_result['new_path'])}")
        if blockage_result['voice_file']:
            print(f"✓ Voice update generated: {blockage_result['voice_file']}")
    
    # Get Flutter update
    print("\n\n" + "=" * 60)
    print("FLUTTER APP DATA:")
    print("=" * 60)
    flutter_data = ai_coordinator.get_flutter_update()
    print(json.dumps(flutter_data, indent=2))

