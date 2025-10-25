import 'package:flutter/material.dart';
import 'dart:math';

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  static const int gridSize = 8;

  // sample positions and walls; replace with live data later
  int userIndex = 3 * gridSize + 4;
  final Set<int> peopleIndices = {10, 22, 55};

  late final List<List<bool>> hWalls; // (gridSize+1) x gridSize
  late final List<List<bool>> vWalls; // gridSize x (gridSize+1)
  List<int> pathToExit = [];

  @override
  void initState() {
    super.initState();
    // initialize all walls closed, then carve a maze
    hWalls = List.generate(gridSize + 1, (_) => List.filled(gridSize, true));
    vWalls = List.generate(gridSize, (_) => List.filled(gridSize + 1, true));
    _generateMaze();
  }

  void _generateMaze() {
    final rnd = Random();
    final visited = List.generate(gridSize, (_) => List.filled(gridSize, false));
    final stack = <List<int>>[];

    final sr = rnd.nextInt(gridSize);
    final sc = rnd.nextInt(gridSize);
    visited[sr][sc] = true;
    stack.add([sr, sc]);

    while (stack.isNotEmpty) {
      final cell = stack.last;
      final r = cell[0];
      final c = cell[1];
      final neighbors = <List<int>>[];
      if (r > 0 && !visited[r - 1][c]) neighbors.add([r - 1, c]);
      if (r < gridSize - 1 && !visited[r + 1][c]) neighbors.add([r + 1, c]);
      if (c > 0 && !visited[r][c - 1]) neighbors.add([r, c - 1]);
      if (c < gridSize - 1 && !visited[r][c + 1]) neighbors.add([r, c + 1]);

      if (neighbors.isNotEmpty) {
        final n = neighbors[rnd.nextInt(neighbors.length)];
        final nr = n[0];
        final nc = n[1];
        if (nr == r - 1) {
          hWalls[r][c] = false;
        } else if (nr == r + 1) {
          hWalls[r + 1][c] = false;
        } else if (nc == c - 1) {
          vWalls[r][c] = false;
        } else if (nc == c + 1) {
          vWalls[r][c + 1] = false;
        }
        visited[nr][nc] = true;
        stack.add([nr, nc]);
      } else {
        stack.removeLast();
      }
    }

    // ensure outer borders remain walls
    for (var c = 0; c < gridSize; c++) hWalls[0][c] = true;
    for (var c = 0; c < gridSize; c++) hWalls[gridSize][c] = true;
    for (var r = 0; r < gridSize; r++) vWalls[r][0] = true;
    for (var r = 0; r < gridSize; r++) vWalls[r][gridSize] = true;

    // create a few exits by opening some outer walls (top, right, bottom, left)
    hWalls[0][3] = false; // top exit at column 3
    vWalls[4][gridSize] = false; // right exit at row 4
    hWalls[gridSize][1] = false; // bottom exit at column 1
    vWalls[2][0] = false; // left exit at row 2

    // compute shortest path from user to nearest exit
    pathToExit = _computePathToNearestExit();
  }

  List<int> _computePathToNearestExit() {
    final int n = gridSize;
    final startR = userIndex ~/ n;
    final startC = userIndex % n;
    final visited = List.generate(n, (_) => List.filled(n, false));
    final prev = List<int>.filled(n * n, -1);
    final q = <List<int>>[];
    visited[startR][startC] = true;
    q.add([startR, startC]);

    int foundR = -1, foundC = -1;

    bool isExitCell(int r, int c) {
      // top
      if (r == 0 && hWalls[0][c] == false) return true;
      // bottom
      if (r == n - 1 && hWalls[n][c] == false) return true;
      // left
      if (c == 0 && vWalls[r][0] == false) return true;
      // right
      if (c == n - 1 && vWalls[r][n] == false) return true;
      return false;
    }

    int idx(int r, int c) => r * n + c;

    while (q.isNotEmpty) {
      final cell = q.removeAt(0);
      final r = cell[0];
      final c = cell[1];
      if (isExitCell(r, c)) {
        foundR = r;
        foundC = c;
        break;
      }

      // neighbors: up
      if (r > 0 && !visited[r - 1][c] && hWalls[r][c] == false) {
        visited[r - 1][c] = true;
        prev[idx(r - 1, c)] = idx(r, c);
        q.add([r - 1, c]);
      }
      // down
      if (r < n - 1 && !visited[r + 1][c] && hWalls[r + 1][c] == false) {
        visited[r + 1][c] = true;
        prev[idx(r + 1, c)] = idx(r, c);
        q.add([r + 1, c]);
      }
      // left
      if (c > 0 && !visited[r][c - 1] && vWalls[r][c] == false) {
        visited[r][c - 1] = true;
        prev[idx(r, c - 1)] = idx(r, c);
        q.add([r, c - 1]);
      }
      // right
      if (c < n - 1 && !visited[r][c + 1] && vWalls[r][c + 1] == false) {
        visited[r][c + 1] = true;
        prev[idx(r, c + 1)] = idx(r, c);
        q.add([r, c + 1]);
      }
    }

    if (foundR == -1) return [];
    // reconstruct
    final path = <int>[];
    var cur = idx(foundR, foundC);
    final startIdx = idx(startR, startC);
    while (cur != -1) {
      path.add(cur);
      if (cur == startIdx) break;
      cur = prev[cur];
    }
    return path.reversed.toList();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text('Map — Floor plan and live robot detections', style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600)),
          const SizedBox(height: 12),
          LayoutBuilder(builder: (context, constraints) {
            final double maxWidth = constraints.maxWidth;
            final double size = min(maxWidth, 360.0);

            return Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                border: Border.all(color: theme.colorScheme.onSurface.withOpacity(0.08)),
                borderRadius: BorderRadius.circular(6),
                color: theme.colorScheme.surface,
              ),
              child: Stack(
                children: [
                  // grid
                  GridView.builder(
                    physics: const NeverScrollableScrollPhysics(),
                    gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: gridSize),
                    itemCount: gridSize * gridSize,
                    itemBuilder: (ctx, index) {
                      final isUser = index == userIndex;
                      final isPerson = peopleIndices.contains(index);

                      return Container(
                        decoration: BoxDecoration(
                          border: Border.all(color: theme.colorScheme.onSurface.withOpacity(0.06)),
                        ),
                        child: Center(
                          child: isUser
                              ? Container(width: 18, height: 18, decoration: BoxDecoration(color: Colors.blue, shape: BoxShape.circle, boxShadow: [BoxShadow(color: Colors.blue.withOpacity(0.3), blurRadius: 6)]))
                              : isPerson
                                  ? Container(width: 14, height: 14, decoration: BoxDecoration(color: Colors.red, shape: BoxShape.circle, boxShadow: [BoxShadow(color: Colors.red.withOpacity(0.25), blurRadius: 4)]))
                                  : const SizedBox.shrink(),
                        ),
                      );
                    },
                  ),

                  // walls overlay
                  CustomPaint(
                    size: Size(size, size),
                    painter: _MazePainter(hWalls: hWalls, vWalls: vWalls, color: theme.colorScheme.onSurface),
                  ),
                  // path overlay
                  if (pathToExit.isNotEmpty)
                    CustomPaint(
                      size: Size(size, size),
                      painter: _PathPainter(path: pathToExit, gridSize: gridSize, color: Colors.green),
                    ),
                ],
              ),
            );
          }),

          const SizedBox(height: 12),
          Row(mainAxisSize: MainAxisSize.min, children: [
            Row(children: [Container(width: 12, height: 12, decoration: const BoxDecoration(color: Colors.blue, shape: BoxShape.circle)), const SizedBox(width: 6), const Text('You')]),
            const SizedBox(width: 16),
            Row(children: [Container(width: 10, height: 10, decoration: const BoxDecoration(color: Colors.red, shape: BoxShape.circle)), const SizedBox(width: 6), const Text('Detected people')]),
          ])
        ],
      ),
    );
  }
}

class _MazePainter extends CustomPainter {
  _MazePainter({required this.hWalls, required this.vWalls, required this.color});

  final List<List<bool>> hWalls;
  final List<List<bool>> vWalls;
  final Color color;

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = color.withOpacity(0.95)
      ..strokeWidth = 8
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.square
      ..isAntiAlias = true;

    final int grid = vWalls.length;
    final cellW = size.width / grid;
    final cellH = size.height / grid;

    // horizontal walls: hWalls has grid+1 rows
    for (var r = 0; r < hWalls.length; r++) {
      for (var c = 0; c < hWalls[r].length; c++) {
        if (!hWalls[r][c]) continue;
        final y = r * cellH;
        final x1 = c * cellW;
        final x2 = (c + 1) * cellW;
        canvas.drawLine(Offset(x1, y), Offset(x2, y), paint);
      }
    }

    // vertical walls
    for (var r = 0; r < vWalls.length; r++) {
      for (var c = 0; c < vWalls[r].length; c++) {
        if (!vWalls[r][c]) continue;
        final x = c * cellW;
        final y1 = r * cellH;
        final y2 = (r + 1) * cellH;
        canvas.drawLine(Offset(x, y1), Offset(x, y2), paint);
      }
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class _PathPainter extends CustomPainter {
  _PathPainter({required this.path, required this.gridSize, required this.color});

  final List<int> path;
  final int gridSize;
  final Color color;

  @override
  void paint(Canvas canvas, Size size) {
    if (path.isEmpty) return;
    final paint = Paint()
      ..color = color
      ..strokeWidth = 4
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..isAntiAlias = true;

    final cellW = size.width / gridSize;
    final cellH = size.height / gridSize;

    final pathPoints = path.map((idx) {
      final r = idx ~/ gridSize;
      final c = idx % gridSize;
      return Offset(c * cellW + cellW / 2, r * cellH + cellH / 2);
    }).toList();

    final pathPainter = Path();
    pathPainter.moveTo(pathPoints.first.dx, pathPoints.first.dy);

    for (var p in pathPoints.skip(1))  {
      pathPainter.lineTo(p.dx, p.dy);
    }
    
    canvas.drawPath(pathPainter, paint);

    // path nodes removed (no dots inside squares) - path is shown as a solid line only
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
