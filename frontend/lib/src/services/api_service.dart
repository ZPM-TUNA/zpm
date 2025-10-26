import 'dart:async';
import 'dart:convert';
import 'package:http/http.dart' as http;

class ApiService {
  // Backend URL - connected to Flask server
  // Later will integrate with ROS2 network for real-time robot data
  static const String baseUrl = 'http://localhost:5001';
  static const bool useMockData = false; // Backend is now running!
  
  // Singleton pattern
  static final ApiService _instance = ApiService._internal();
  factory ApiService() => _instance;
  ApiService._internal();

  // Health check
  Future<Map<String, dynamic>> healthCheck() async {
    if (useMockData) {
      return {
        'status': 'running',
        'service': 'evacuation-coordinator',
        'maze_size': 8,
        'robots': 2,
        'humans': 3,
        'exits': 2
      };
    }
    
    try {
      final response = await http.get(Uri.parse('$baseUrl/health'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Health check failed');
    } catch (e) {
      throw Exception('Cannot connect to backend: $e');
    }
  }

  // Get maze state
  Future<Map<String, dynamic>> getMazeState() async {
    if (useMockData) {
      return {
        'size': 8,
        'robots': {
          'robot_1': [2, 3],
          'robot_2': [6, 1]
        },
        'humans': {
          'human_1': [4, 4],
          'human_2': [5, 3],
          'human_3': [6, 6]
        },
        'obstacles': [
          [2, 2], [2, 3], [3, 2], [5, 5], [5, 6]
        ],
        'exits': [
          [0, 7], [7, 7]
        ]
      };
    }
    
    try {
      final response = await http.get(Uri.parse('$baseUrl/api/maze/state'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to get maze state');
    } catch (e) {
      throw Exception('Error fetching maze state: $e');
    }
  }

  // Get complete state
  Future<Map<String, dynamic>> getState() async {
    if (useMockData) {
      return {
        'time': 10.5,
        'maze_size': 8,
        'robots': {
          'robot_1': {'position': [2.5, 3.2], 'explored': 15, 'path': []},
          'robot_2': {'position': [6.1, 1.8], 'explored': 12, 'path': []}
        },
        'humans': {
          'human_1': [4, 4],
          'human_2': [5, 3]
        },
        'obstacles': [[2, 2], [2, 3], [5, 5]],
        'exits': [[0, 7], [7, 7]],
        'evacuation_plans': {},
        'stats': {
          'humans_detected': 2,
          'obstacles_detected': 3,
          'total_humans': 3
        }
      };
    }
    
    try {
      final response = await http.get(Uri.parse('$baseUrl/api/state'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to get state');
    } catch (e) {
      throw Exception('Error fetching state: $e');
    }
  }

  // Get Flutter update (complete state)
  Future<Map<String, dynamic>> getFlutterUpdate() async {
    if (useMockData) {
      return {
        'robots': [
          {
            'id': 'scout_1',
            'position': [2, 3],
            'status': 'exploring',
            'battery': 85,
            'explored_area': [[2, 3], [3, 3], [4, 3], [4, 4]]
          },
          {
            'id': 'scout_2',
            'position': [6, 1],
            'status': 'mapping',
            'battery': 92,
            'explored_area': [[6, 1], [6, 2], [7, 2]]
          }
        ],
        'humans': [
          {
            'id': 'person_1',
            'position': [4, 4],
            'evacuation_path': [[4, 4], [3, 4], [2, 5], [1, 6], [0, 7]],
            'priority': 'high',
            'distance_to_exit': 5,
            'exit_target': [0, 7]
          },
          {
            'id': 'person_2',
            'position': [5, 3],
            'evacuation_path': [[5, 3], [6, 4], [7, 5], [7, 6], [7, 7]],
            'priority': 'medium',
            'distance_to_exit': 5,
            'exit_target': [7, 7]
          },
          {
            'id': 'person_3',
            'position': [6, 6],
            'evacuation_path': [[6, 6], [7, 6], [7, 7]],
            'priority': 'low',
            'distance_to_exit': 2,
            'exit_target': [7, 7]
          }
        ],
        'guidance': {
          'text': 'Person at [4,4]: Walk 3 steps west, then 3 steps north to northwest exit. Person at [5,3]: Walk 2 steps east, then 4 steps north to northeast exit. Person at [6,6]: Walk 1 step east, then 1 step north to exit. Avoid obstacles at [2,2] and [5,5].',
          'voice_file': null,
          'timestamp': DateTime.now().toIso8601String()
        },
        'maze': {
          'size': 8,
          'obstacles': [[2, 2], [2, 3], [3, 2], [5, 5], [5, 6]],
          'exits': [[0, 7], [7, 7]]
        }
      };
    }
    
    try {
      final response = await http.get(Uri.parse('$baseUrl/api/flutter-update'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to get Flutter update');
    } catch (e) {
      throw Exception('Error fetching update: $e');
    }
  }

  // Get AI guidance
  Future<Map<String, dynamic>> getAIGuidance() async {
    if (useMockData) {
      return {
        'guidance': 'All evacuation paths calculated. Robots are guiding humans to nearest exits.',
        'timestamp': DateTime.now().millisecondsSinceEpoch / 1000
      };
    }
    
    try {
      final response = await http.get(Uri.parse('$baseUrl/api/ai-guidance'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to get AI guidance');
    } catch (e) {
      throw Exception('Error fetching AI guidance: $e');
    }
  }

  // Control: Start simulation
  Future<Map<String, dynamic>> startSimulation() async {
    if (useMockData) {
      return {'success': true, 'message': 'Simulation started'};
    }
    
    try {
      final response = await http.post(Uri.parse('$baseUrl/api/control/start'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to start simulation');
    } catch (e) {
      throw Exception('Error starting simulation: $e');
    }
  }

  // Control: Stop simulation
  Future<Map<String, dynamic>> stopSimulation() async {
    if (useMockData) {
      return {'success': true, 'message': 'Simulation stopped'};
    }
    
    try {
      final response = await http.post(Uri.parse('$baseUrl/api/control/stop'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to stop simulation');
    } catch (e) {
      throw Exception('Error stopping simulation: $e');
    }
  }

  // Control: Reset simulation
  Future<Map<String, dynamic>> resetSimulation() async {
    if (useMockData) {
      return {'success': true, 'message': 'Simulation reset'};
    }
    
    try {
      final response = await http.post(Uri.parse('$baseUrl/api/control/reset'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to reset simulation');
    } catch (e) {
      throw Exception('Error resetting simulation: $e');
    }
  }

  // Get stats
  Future<Map<String, dynamic>> getStats() async {
    if (useMockData) {
      return {
        'simulation_time': 45.2,
        'total_robots': 2,
        'total_humans': 3,
        'humans_detected': 2,
        'humans_with_evacuation_paths': 2,
        'total_obstacles': 5
      };
    }
    
    try {
      final response = await http.get(Uri.parse('$baseUrl/api/stats'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to get stats');
    } catch (e) {
      throw Exception('Error fetching stats: $e');
    }
  }

  // Analyze evacuation
  Future<Map<String, dynamic>> analyzeEvacuation({bool generateVoice = false}) async {
    if (useMockData) {
      await Future.delayed(const Duration(milliseconds: 800));
      return {
        'success': true,
        'guidance': 'Evacuation paths calculated. Person at [4,4]: Walk west 3 steps, then north 3 steps to exit. Person at [5,3]: Walk east 2 steps, then north 4 steps. Person at [6,6]: Walk to nearest exit at [7,7]. Scouts have mapped safe routes.',
        'voice_file': null,
        'humans_in_danger': 3,
        'calculated_paths': {
          'person_1': [[4,4], [3,4], [2,5], [1,6], [0,7]],
          'person_2': [[5,3], [6,4], [7,5], [7,6], [7,7]],
          'person_3': [[6,6], [7,6], [7,7]]
        },
        'evacuation_status': 'paths_ready'
      };
    }
    
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/api/evacuation/analyze'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({'generate_voice': generateVoice}),
      );
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to analyze evacuation');
    } catch (e) {
      throw Exception('Error analyzing evacuation: $e');
    }
  }

  // Send robot command
  Future<Map<String, dynamic>> sendRobotCommand(
    String robotId,
    String command, {
    int speed = 200,
    int durationMs = 0,
  }) async {
    if (useMockData) {
      await Future.delayed(const Duration(milliseconds: 300));
      return {
        'success': true,
        'robot_id': robotId,
        'command': command
      };
    }
    
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/api/elegoo/command'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({
          'robot_id': robotId,
          'command': command,
          'speed': speed,
          'duration_ms': durationMs,
        }),
      );
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to send command');
    } catch (e) {
      throw Exception('Error sending command: $e');
    }
  }

  // Get robot sensors
  Future<Map<String, dynamic>> getRobotSensors(String robotId) async {
    if (useMockData) {
      return {
        'success': true,
        'robot_id': robotId,
        'sensors': {
          'ultrasonic_cm': 25.5,
          'line_tracking': {
            'left': 0,
            'middle': 1,
            'right': 0
          }
        }
      };
    }
    
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/api/elegoo/sensors/$robotId'),
      );
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to get sensors');
    } catch (e) {
      throw Exception('Error fetching sensors: $e');
    }
  }

  // Setup demo scenario
  Future<Map<String, dynamic>> setupDemo() async {
    if (useMockData) {
      await Future.delayed(const Duration(milliseconds: 500));
      return {
        'success': true,
        'message': 'Demo scenario initialized',
        'guidance': 'Demo environment ready. 2 robots and 3 humans positioned.',
      };
    }
    
    try {
      final response = await http.post(Uri.parse('$baseUrl/api/demo/setup'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to setup demo');
    } catch (e) {
      throw Exception('Error setting up demo: $e');
    }
  }

  // Calculate path
  Future<Map<String, dynamic>> calculatePath(
    List<int> start,
    List<int> goal,
  ) async {
    if (useMockData) {
      await Future.delayed(const Duration(milliseconds: 400));
      return {
        'success': true,
        'path': [start, [start[0] + 1, start[1]], [goal[0], start[1]], goal],
        'length': 4
      };
    }
    
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/api/pathfinding/calculate'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({
          'start': start,
          'goal': goal,
        }),
      );
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
      throw Exception('Failed to calculate path');
    } catch (e) {
      throw Exception('Error calculating path: $e');
    }
  }
}

