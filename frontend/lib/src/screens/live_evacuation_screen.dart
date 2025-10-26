import 'package:flutter/material.dart';
import 'dart:async';
import '../services/api_service.dart';

class LiveEvacuationScreen extends StatefulWidget {
  const LiveEvacuationScreen({super.key});

  @override
  State<LiveEvacuationScreen> createState() => _LiveEvacuationScreenState();
}

class _LiveEvacuationScreenState extends State<LiveEvacuationScreen> {
  final ApiService _api = ApiService();
  Timer? _updateTimer;
  Map<String, dynamic>? _currentState;
  bool _loading = true;
  String? _error;
  int _mazeSize = 8;

  @override
  void initState() {
    super.initState();
    _loadState();
    // Update every 200ms for smooth animation
    _updateTimer = Timer.periodic(const Duration(milliseconds: 200), (_) => _loadState());
  }

  @override
  void dispose() {
    _updateTimer?.cancel();
    super.dispose();
  }

  Future<void> _loadState() async {
    try {
      final state = await _api.getFlutterUpdate();
      if (mounted) {
        setState(() {
          _currentState = state;
          _loading = false;
          _error = null;
          if (state['maze'] != null && state['maze']['size'] != null) {
            _mazeSize = state['maze']['size'] as int;
          }
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _loading = false;
          _error = e.toString();
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_loading && _currentState == null) {
      return const Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            CircularProgressIndicator(),
            SizedBox(height: 16),
            Text('Connecting to evacuation system...'),
          ],
        ),
      );
    }

    if (_error != null && _currentState == null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.error_outline, size: 64, color: Colors.red),
            const SizedBox(height: 16),
            Text('Error: $_error'),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: _loadState,
              child: const Text('Retry'),
            ),
          ],
        ),
      );
    }

    return SingleChildScrollView(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _buildHeader(),
            const SizedBox(height: 16),
            _buildMazeGrid(),
            const SizedBox(height: 16),
            _buildStats(),
            const SizedBox(height: 16),
            _buildAIGuidance(),
            const SizedBox(height: 16),
            _buildControls(),
          ],
        ),
      ),
    );
  }

  Widget _buildHeader() {
    final systemStatus = _currentState?['system_status'] ?? {};
    final isRunning = systemStatus['simulation_running'] ?? false;
    final time = (systemStatus['time'] ?? 0.0).toDouble();

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Live Evacuation System',
                  style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 4),
                Text(
                  'Time: ${time.toStringAsFixed(1)}s',
                  style: TextStyle(color: Colors.grey[600]),
                ),
              ],
            ),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: isRunning ? Colors.green : Colors.orange,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(
                    isRunning ? Icons.play_arrow : Icons.pause,
                    size: 16,
                    color: Colors.white,
                  ),
                  const SizedBox(width: 4),
                  Text(
                    isRunning ? 'Running' : 'Paused',
                    style: const TextStyle(color: Colors.white),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMazeGrid() {
    final maze = _currentState?['maze'] ?? {};
    final obstacles = (maze['obstacles'] as List?)?.cast<List>() ?? [];
    final exits = (maze['exits'] as List?)?.cast<List>() ?? [];
    final robots = (_currentState?['robots'] as List?)?.cast<Map<String, dynamic>>() ?? [];
    final humans = (_currentState?['humans'] as List?)?.cast<Map<String, dynamic>>() ?? [];

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            const Text(
              'Maze - Live View',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 16),
            AspectRatio(
              aspectRatio: 1,
              child: Container(
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.grey[400]!),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: GridView.builder(
                  physics: const NeverScrollableScrollPhysics(),
                  gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: _mazeSize,
                  ),
                  itemCount: _mazeSize * _mazeSize,
                  itemBuilder: (context, index) {
                    final x = index % _mazeSize;
                    final y = _mazeSize - 1 - (index ~/ _mazeSize); // Flip Y for proper display

                    return _buildCell(x, y, obstacles, exits, robots, humans);
                  },
                ),
              ),
            ),
            const SizedBox(height: 12),
            _buildLegend(),
          ],
        ),
      ),
    );
  }

  Widget _buildCell(
    int x,
    int y,
    List<List> obstacles,
    List<List> exits,
    List<Map<String, dynamic>> robots,
    List<Map<String, dynamic>> humans,
  ) {
    // Check what's at this position
    final isObstacle = obstacles.any((obs) => obs[0] == x && obs[1] == y);
    final isExit = exits.any((exit) => exit[0] == x && exit[1] == y);
    
    // Check robots (they have floating point positions)
    final robotAtPos = robots.where((robot) {
      final pos = robot['position'] as List;
      return pos[0].round() == x && pos[1].round() == y;
    }).toList();
    
    // Check humans (they have integer positions)
    final humanAtPos = humans.where((human) {
      final pos = human['position'] as List;
      return pos[0] == x && pos[1] == y;
    }).toList();

    // Check if any evacuation path passes through this cell
    bool isOnEvacuationPath = humans.any((human) {
      final path = human['evacuation_path'] as List?;
      if (path == null) return false;
      return path.any((point) {
        final p = point as List;
        return p[0] == x && p[1] == y;
      });
    });

    Color cellColor = Colors.white;
    Widget? icon;
    Color? borderColor;

    if (isObstacle) {
      // Obstacles are BLACK
      cellColor = Colors.black;
    } else if (isExit) {
      // Exit is BRIGHT GREEN with icon
      cellColor = Colors.green[600]!;
      icon = const Icon(Icons.exit_to_app, color: Colors.white, size: 20);
    } else if (isOnEvacuationPath) {
      // Evacuation path is BRIGHT GREEN (before robots/humans overlay)
      cellColor = Colors.green[300]!;
    } else {
      // Empty cells are white
      cellColor = Colors.white;
    }
    
    // Overlay robots and humans on top
    if (robotAtPos.isNotEmpty) {
      // Robots are RED with robot icon
      icon = const Icon(Icons.smart_toy, color: Colors.red, size: 24);
    } else if (humanAtPos.isNotEmpty) {
      // Humans are BLUE DOT
      icon = Container(
        width: 16,
        height: 16,
        decoration: const BoxDecoration(
          color: Colors.blue,
          shape: BoxShape.circle,
        ),
      );
    }

    return Container(
      decoration: BoxDecoration(
        color: cellColor,
        border: Border.all(color: Colors.grey[400]!, width: 1),
      ),
      child: Center(child: icon),
    );
  }

  Widget _buildLegend() {
    return Wrap(
      spacing: 16,
      runSpacing: 8,
      alignment: WrapAlignment.center,
      children: [
        _legendItem(Colors.blue, 'Robot', Icons.smart_toy),
        _legendItem(Colors.red, 'Human', Icons.person),
        _legendItem(Colors.green, 'Exit', Icons.exit_to_app),
        _legendItem(Colors.grey[800]!, 'Obstacle', null),
        _legendItem(Colors.orange[200]!, 'Path', null),
      ],
    );
  }

  Widget _legendItem(Color color, String label, IconData? icon) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 20,
          height: 20,
          decoration: BoxDecoration(
            color: color,
            border: Border.all(color: Colors.grey),
            borderRadius: BorderRadius.circular(4),
          ),
          child: icon != null ? Icon(icon, size: 12, color: Colors.white) : null,
        ),
        const SizedBox(width: 4),
        Text(label, style: const TextStyle(fontSize: 12)),
      ],
    );
  }

  Widget _buildStats() {
    final stats = _currentState?['stats'] ?? {};
    final systemStatus = _currentState?['system_status'] ?? {};

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Statistics',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 12),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _statItem(
                  '${systemStatus['robots_active'] ?? 0}',
                  'Robots Active',
                  Icons.smart_toy,
                  Colors.blue,
                ),
                _statItem(
                  '${systemStatus['humans_detected'] ?? 0}/${stats['total_humans'] ?? 0}',
                  'Humans Detected',
                  Icons.person,
                  Colors.red,
                ),
                _statItem(
                  '${systemStatus['humans_with_paths'] ?? 0}',
                  'Evacuation Paths',
                  Icons.route,
                  Colors.orange,
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _statItem(String value, String label, IconData icon, Color color) {
    return Column(
      children: [
        CircleAvatar(
          backgroundColor: color.withOpacity(0.2),
          child: Icon(icon, color: color),
        ),
        const SizedBox(height: 8),
        Text(
          value,
          style: const TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
        ),
        Text(
          label,
          style: TextStyle(fontSize: 12, color: Colors.grey[600]),
          textAlign: TextAlign.center,
        ),
      ],
    );
  }

  Widget _buildAIGuidance() {
    final guidance = _currentState?['ai_guidance'] as String?;

    if (guidance == null || guidance.isEmpty) {
      return const SizedBox.shrink();
    }

    return Card(
      color: Colors.amber[50],
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.lightbulb, color: Colors.amber[700]),
                const SizedBox(width: 8),
                const Text(
                  'AI Guidance',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Text(
              guidance,
              style: const TextStyle(fontSize: 14),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildControls() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const Text(
              'Controls',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 12),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                ElevatedButton.icon(
                  onPressed: () async {
                    try {
                      await _api.startSimulation();
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(content: Text('Simulation started')),
                      );
                    } catch (e) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('Error: $e')),
                      );
                    }
                  },
                  icon: const Icon(Icons.play_arrow),
                  label: const Text('Start'),
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.green),
                ),
                ElevatedButton.icon(
                  onPressed: () async {
                    try {
                      await _api.stopSimulation();
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(content: Text('Simulation stopped')),
                      );
                    } catch (e) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('Error: $e')),
                      );
                    }
                  },
                  icon: const Icon(Icons.pause),
                  label: const Text('Stop'),
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.orange),
                ),
                ElevatedButton.icon(
                  onPressed: () async {
                    try {
                      await _api.resetSimulation();
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(content: Text('Simulation reset')),
                      );
                    } catch (e) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('Error: $e')),
                      );
                    }
                  },
                  icon: const Icon(Icons.refresh),
                  label: const Text('Reset'),
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.blue),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

