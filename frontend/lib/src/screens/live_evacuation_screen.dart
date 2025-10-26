import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math';
import 'dart:ui' as ui;
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
            // 8x8 Maze Grid - Fit on screen without scrolling
            Center(
              child: ConstrainedBox(
                constraints: BoxConstraints(
                  maxWidth: 350,
                  maxHeight: 350,
                ),
                child: AspectRatio(
                  aspectRatio: 1.0,
                  child: Container(
                    decoration: BoxDecoration(
                      border: Border.all(color: Colors.black, width: 3),
                      color: Colors.grey[100],
                    ),
                    child: CustomPaint(
                      painter: PathPainter(humans: humans),
                      child: GridView.builder(
                        physics: const NeverScrollableScrollPhysics(),
                        padding: EdgeInsets.zero,
                        gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
                          crossAxisCount: 8,
                          mainAxisSpacing: 1,
                          crossAxisSpacing: 1,
                        ),
                        itemCount: 64,
                        itemBuilder: (context, index) {
                          final x = index % 8;
                          final y = 7 - (index ~/ 8);

                          return _buildCell(x, y, obstacles, exits, robots, humans);
                        },
                      ),
                    ),
                  ),
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
    Widget? pathIndicator;

    // Base layer
    if (isObstacle) {
      cellColor = Colors.black;
    } else if (isExit) {
      cellColor = Colors.green[700]!;
      icon = const Icon(Icons.exit_to_app, color: Colors.white, size: 28);
    } else {
      cellColor = Colors.white;
    }
    
    // Don't draw path dots here - PathPainter will draw lines
    
    // Overlay robots and humans on top
    if (robotAtPos.isNotEmpty) {
      icon = const Icon(Icons.smart_toy, color: Colors.red, size: 32);
    } else if (humanAtPos.isNotEmpty) {
      icon = Container(
        width: 20,
        height: 20,
        decoration: const BoxDecoration(
          color: Colors.blue,
          shape: BoxShape.circle,
        ),
      );
    }

    return Container(
      decoration: BoxDecoration(
        color: cellColor,
        border: Border.all(color: Colors.grey[300]!, width: 0.5),
      ),
      child: Stack(
        children: [
          if (pathIndicator != null) pathIndicator,
          if (icon != null) Center(child: icon),
        ],
      ),
    );
  }

  Widget _buildLegend() {
    return Wrap(
      spacing: 16,
      runSpacing: 8,
      alignment: WrapAlignment.center,
      children: [
        _legendItem(Colors.red, 'Robot', Icons.smart_toy),
        _legendItem(Colors.blue, 'Human', Icons.circle),
        _legendItem(Colors.green[700]!, 'Exit', Icons.exit_to_app),
        _legendItem(Colors.black, 'Obstacle', Icons.block),
        _legendItem(Colors.green[600]!, 'Path (green dots)', Icons.circle),
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

// Custom painter to draw evacuation path as bold dotted lines
class PathPainter extends CustomPainter {
  final List<Map<String, dynamic>> humans;

  PathPainter({required this.humans});

  @override
  void paint(Canvas canvas, Size size) {
    final cellSize = size.width / 8;
    
    for (var human in humans) {
      final path = human['evacuation_path'] as List?;
      if (path == null || path.isEmpty) continue;
      
      final paint = Paint()
        ..color = Colors.green  // BRIGHT GREEN
        ..strokeWidth = 12.0  // VERY THICK line
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round;
      
      // Draw THICK GREEN SNAKE PATH
      for (int i = 0; i < path.length - 1; i++) {
        final current = path[i] as List;
        final next = path[i + 1] as List;
        
        // Convert grid coordinates to screen coordinates
        final x1 = (current[0] + 0.5) * cellSize;
        final y1 = (7 - current[1] + 0.5) * cellSize;
        final x2 = (next[0] + 0.5) * cellSize;
        final y2 = (7 - next[1] + 0.5) * cellSize;
        
        // Draw glow effect (outer line)
        final glowPaint = Paint()
          ..color = Colors.green.withOpacity(0.3)
          ..strokeWidth = 20.0
          ..style = PaintingStyle.stroke
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round;
        canvas.drawLine(Offset(x1, y1), Offset(x2, y2), glowPaint);
        
        // Draw main thick line
        canvas.drawLine(Offset(x1, y1), Offset(x2, y2), paint);
      }
    }
  }
  
  void _drawDottedLine(Canvas canvas, Offset start, Offset end, Paint paint) {
    const dashWidth = 8.0;
    const dashSpace = 6.0;
    
    final dx = end.dx - start.dx;
    final dy = end.dy - start.dy;
    final distance = sqrt(dx * dx + dy * dy);
    final dashCount = (distance / (dashWidth + dashSpace)).floor();
    
    for (int i = 0; i < dashCount; i++) {
      final t1 = (i * (dashWidth + dashSpace)) / distance;
      final t2 = ((i * (dashWidth + dashSpace)) + dashWidth) / distance;
      
      final p1 = Offset(
        start.dx + dx * t1,
        start.dy + dy * t1,
      );
      final p2 = Offset(
        start.dx + dx * t2,
        start.dy + dy * t2,
      );
      
      canvas.drawLine(p1, p2, paint);
    }
  }

  @override
  bool shouldRepaint(PathPainter oldDelegate) => true;
}

