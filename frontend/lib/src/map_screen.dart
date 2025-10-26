import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';

// This file implements the GUI-only simulation port of backend/simulation.py.
// It intentionally replaces the old maze-based map with walls that occupy
// entire grid squares and runs a stepwise simulation with two robots.

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  static const int gridSize = 8;

  static const List<List<int>> simMaze = [
    [0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 0, 0],
  ];

  final Set<int> blockedCells = {};
  final Set<int> randomObstacles = {};

  late List<SimRobot> robots;
  final Point<int> goal = const Point(0, 0);

  Timer? _simTimer;

  @override
  void initState() {
    super.initState();
    _initSimulation();
  }

  @override
  void dispose() {
    _simTimer?.cancel();
    super.dispose();
  }

  void _initSimulation() {
    blockedCells.clear();
    for (var r = 0; r < gridSize; r++) {
      for (var c = 0; c < gridSize; c++) {
        if (simMaze[r][c] == 1) blockedCells.add(r * gridSize + c);
      }
    }

    _generateRandomObstacles(5);

    final redStart = Point(7, 0);
    final blueStart = Point(0, 7);

    robots = [
      SimRobot(id: 1, pos: redStart, goal: goal, color: Colors.red),
      SimRobot(id: 2, pos: blueStart, goal: goal, color: Colors.blue),
    ];

    for (final r in robots) {
      final p = _astarPath(r.pos, r.goal, blockedCells);
      if (p.isNotEmpty) r.path = p;
      else r.finished = true;
    }

    setState(() {});
  }

  void _generateRandomObstacles(int num) {
    randomObstacles.clear();
    final forbidden = <int>{};
    forbidden.addAll([7 * gridSize + 0, 0 * gridSize + 7, goal.x * gridSize + goal.y]);
    forbidden.addAll(blockedCells);

    final rnd = Random();
    int attempts = 0;
    while (randomObstacles.length < num && attempts < 200) {
      final r = rnd.nextInt(gridSize);
      final c = rnd.nextInt(gridSize);
      final idx = r * gridSize + c;
      if (!forbidden.contains(idx) && !randomObstacles.contains(idx)) randomObstacles.add(idx);
      attempts++;
    }
  }

  List<Point<int>> _astarPath(Point<int> start, Point<int> goal, Set<int> blockedSet) {
    final int n = gridSize;
    final int startIdx = start.x * n + start.y;
    final int goalIdx = goal.x * n + goal.y;
    if (startIdx == goalIdx) return [start];

    int idxOf(int r, int c) => r * n + c;

    final openSet = <int>{startIdx};
    final cameFrom = <int, int>{};

    final gScore = List<double>.filled(n * n, double.infinity);
    final fScore = List<double>.filled(n * n, double.infinity);

    gScore[startIdx] = 0;
    fScore[startIdx] = _manhattan(startIdx, goalIdx);

    while (openSet.isNotEmpty) {
      final current = openSet.reduce((a, b) => fScore[a] < fScore[b] ? a : b);
      if (current == goalIdx) {
        final path = <Point<int>>[];
        var cur = current;
        while (true) {
          final r = cur ~/ n;
          final c = cur % n;
          path.add(Point(r, c));
          if (!cameFrom.containsKey(cur)) break;
          cur = cameFrom[cur]!;
        }
        return path.reversed.toList();
      }

      openSet.remove(current);
      final cr = current ~/ n;
      final cc = current % n;

      final neighbors = <int>[];
      if (cr > 0) neighbors.add(idxOf(cr - 1, cc));
      if (cr < n - 1) neighbors.add(idxOf(cr + 1, cc));
      if (cc > 0) neighbors.add(idxOf(cr, cc - 1));
      if (cc < n - 1) neighbors.add(idxOf(cr, cc + 1));

      for (final neigh in neighbors) {
        if (blockedSet.contains(neigh)) continue;
        final tentativeG = gScore[current] + 1;
        if (tentativeG < gScore[neigh]) {
          cameFrom[neigh] = current;
          gScore[neigh] = tentativeG;
          fScore[neigh] = tentativeG + _manhattan(neigh, goalIdx);
          openSet.add(neigh);
        }
      }
    }
    return [];
  }

  double _manhattan(int a, int b) {
    final ar = a ~/ gridSize;
    final ac = a % gridSize;
    final br = b ~/ gridSize;
    final bc = b % gridSize;
    return (ar - br).abs().toDouble() + (ac - bc).abs().toDouble();
  }

  void _startSimulation() {
    if (_simTimer != null && _simTimer!.isActive) return;

    for (final r in robots) {
      final p = _astarPath(r.pos, r.goal, blockedCells);
      if (p.isNotEmpty) {
        r.path = p;
        r.finished = false;
      } else {
        r.path = [];
        r.finished = true;
      }
    }

    _simTimer = Timer.periodic(const Duration(milliseconds: 500), (_) {
      _simStep();
    });
    setState(() {});
  }

  void _stopSimulation() {
    _simTimer?.cancel();
    _simTimer = null;
    setState(() {});
  }

  void _simStep() {
    for (final robot in robots) {
      if (!robot.finished) robot.step(randomObstacles, blockedCells, _astarPath);
    }
    if (mounted) setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text('Simulation — robots discover obstacles and replan', style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600)),
          const SizedBox(height: 12),
          LayoutBuilder(builder: (context, constraints) {
            final double maxWidth = constraints.maxWidth;
            final double size = min(maxWidth, 420.0);

            return Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                border: Border.all(color: theme.colorScheme.onSurface.withOpacity(0.12)),
                borderRadius: BorderRadius.circular(6),
                color: theme.colorScheme.surface,
              ),
              child: Stack(
                children: [
                  GridView.builder(
                    physics: const NeverScrollableScrollPhysics(),
                    padding: EdgeInsets.zero,
                    gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: gridSize),
                    itemCount: gridSize * gridSize,
                    itemBuilder: (ctx, index) {
                      final isBlocked = blockedCells.contains(index);
                      return Container(
                        decoration: BoxDecoration(
                          border: Border.all(color: theme.colorScheme.onSurface.withOpacity(0.06)),
                          color: isBlocked ? theme.colorScheme.onSurface.withOpacity(0.08) : Colors.transparent,
                        ),
                      );
                    },
                  ),

                  CustomPaint(
                    size: Size(size, size),
                    painter: _SimPainter(blockedCells: blockedCells, randomObstacles: randomObstacles, robots: robots, goal: goal),
                  ),
                ],
              ),
            );
          }),

          const SizedBox(height: 12),

          Wrap(
            spacing: 12,
            runSpacing: 8,
            alignment: WrapAlignment.center,
            children: [
              ElevatedButton(
                onPressed: () {
                  if (_simTimer == null) _startSimulation();
                  else _stopSimulation();
                },
                child: Text(_simTimer == null ? 'Run simulation' : 'Stop simulation'),
              ),

              ElevatedButton(
                onPressed: () {
                  _generateRandomObstacles(5);
                  for (final r in robots) {
                    final p = _astarPath(r.pos, r.goal, blockedCells);
                    if (p.isNotEmpty) r.path = p;
                  }
                  setState(() {});
                },
                child: const Text('Regenerate obstacles'),
              ),

              ElevatedButton(
                onPressed: () {
                  // stop any running timer and re-initialize the entire simulation
                  _stopSimulation();
                  _initSimulation();
                },
                child: const Text('Restart'),
              ),

              Padding(
                padding: const EdgeInsets.only(left: 4.0, top: 6.0),
                child: Text(_simTimer == null ? 'Stopped' : 'Running', style: TextStyle(color: _simTimer == null ? Colors.grey : Colors.green, fontWeight: FontWeight.w600)),
              ),
            ],
          ),

          const SizedBox(height: 12),

          Wrap(spacing: 16, runSpacing: 8, alignment: WrapAlignment.center, children: [
            Row(mainAxisSize: MainAxisSize.min, children: [Container(width: 12, height: 12, decoration: const BoxDecoration(color: Colors.blue, shape: BoxShape.circle)), const SizedBox(width: 6), const Text('Red robot')]),
            Row(mainAxisSize: MainAxisSize.min, children: [Container(width: 12, height: 12, decoration: const BoxDecoration(color: Colors.red, shape: BoxShape.circle)), const SizedBox(width: 6), const Text('Blue robot')]),
            Row(mainAxisSize: MainAxisSize.min, children: [Container(width: 12, height: 12, decoration: BoxDecoration(color: Colors.green[700], borderRadius: BorderRadius.circular(3))), const SizedBox(width: 6), const Text('Goal')]),
            Row(mainAxisSize: MainAxisSize.min, children: [Container(width: 12, height: 12, decoration: BoxDecoration(color: Colors.red[700], borderRadius: BorderRadius.circular(3))), const SizedBox(width: 6), const Text('Random obstacle')]),
          ])
        ],
      ),
    );
  }
}

class SimRobot {
  SimRobot({required this.id, required this.pos, required this.goal, required this.color}) {
    trail = [pos];
  }

  final int id;
  Point<int> pos; // x=row, y=col
  final Point<int> goal;
  List<Point<int>> path = [];
  bool finished = false;
  late List<Point<int>> trail;
  final Color color;
  final Set<int> discoveredObstacles = {};

  bool step(Set<int> randomObstacles, Set<int> staticBlocked, List<Point<int>> Function(Point<int>, Point<int>, Set<int>) astar) {
    if (finished) return false;
    if (path.isEmpty) return false;

    if (path.length > 1) {
      final next = path[1];
      final nextIdx = next.x * _MapScreenState.gridSize + next.y;

      if (randomObstacles.contains(nextIdx)) {
        discoveredObstacles.add(nextIdx);
        final blocked = <int>{}..addAll(staticBlocked)..addAll(discoveredObstacles);
        final newPath = astar(Point(pos.x, pos.y), Point(goal.x, goal.y), blocked);
        if (newPath.isEmpty) {
          finished = true;
          return false;
        }
        path = newPath;
        return true;
      }

      path.removeAt(0);
      final movedTo = path[0];
      pos = Point(movedTo.x, movedTo.y);
      trail.add(pos);
      if (pos == goal) finished = true;
      return true;
    } else {
      finished = true;
      return false;
    }
  }
}

class _SimPainter extends CustomPainter {
  _SimPainter({required this.blockedCells, required this.randomObstacles, required this.robots, required this.goal});

  final Set<int> blockedCells;
  final Set<int> randomObstacles;
  final List<SimRobot> robots;
  final Point<int> goal;

  @override
  void paint(Canvas canvas, Size size) {
    final cellW = size.width / _MapScreenState.gridSize;
    final cellH = size.height / _MapScreenState.gridSize;

    final blockedPaint = Paint()..color = Colors.black.withOpacity(0.08);
    for (final idx in blockedCells) {
      final r = idx ~/ _MapScreenState.gridSize;
      final c = idx % _MapScreenState.gridSize;
      final rect = Rect.fromLTWH(c * cellW, r * cellH, cellW, cellH);
      canvas.drawRect(rect, blockedPaint);
    }

    final obstaclePaint = Paint()
      ..color = Colors.red[700]!
      ..strokeWidth = 2.0
      ..style = PaintingStyle.stroke;

    for (final idx in randomObstacles) {
      final r = idx ~/ _MapScreenState.gridSize;
      final c = idx % _MapScreenState.gridSize;
      final x1 = c * cellW + cellW * 0.18;
      final y1 = r * cellH + cellH * 0.18;
      final x2 = c * cellW + cellW * 0.82;
      final y2 = r * cellH + cellH * 0.82;
      canvas.drawLine(Offset(x1, y1), Offset(x2, y2), obstaclePaint);
      canvas.drawLine(Offset(x1, y2), Offset(x2, y1), obstaclePaint);
    }

    final goalPaint = Paint()..color = Colors.green[700]!;
    canvas.drawRect(Rect.fromLTWH(goal.y * cellW + cellW * 0.18, goal.x * cellH + cellH * 0.18, cellW * 0.64, cellH * 0.64), goalPaint);

    for (final robot in robots) {
      final trailPaint = Paint()..color = robot.color.withOpacity(0.22);
      for (final p in robot.trail) {
        final rect = Rect.fromLTWH(p.y * cellW + cellW * 0.06, p.x * cellH + cellH * 0.06, cellW * 0.88, cellH * 0.88);
        canvas.drawRect(rect, trailPaint);
      }
    }

    for (final robot in robots) {
      if (robot.finished) continue;
      final cx = robot.pos.y * cellW + cellW / 2;
      final cy = robot.pos.x * cellH + cellH / 2;
      final paint = Paint()..color = robot.color;
      canvas.drawCircle(Offset(cx, cy), min(cellW, cellH) * 0.18, paint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}


