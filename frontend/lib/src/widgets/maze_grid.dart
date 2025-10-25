import 'package:flutter/material.dart';
import 'package:frontend/src/models/evacuation_state.dart';

class MazeGrid extends StatelessWidget {
  final MazeData maze;
  final List<RobotData> robots;
  final List<HumanData> humans;

  const MazeGrid({
    super.key,
    required this.maze,
    required this.robots,
    required this.humans,
  });

  @override
  Widget build(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final maxGridWidth = screenWidth > 600 ? 500.0 : screenWidth - 48;
    final cellSize = maxGridWidth / maze.size;

    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        border: Border.all(color: Colors.grey.shade400, width: 2),
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 8,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(10),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: List.generate(maze.size, (row) {
            return Row(
              mainAxisSize: MainAxisSize.min,
              children: List.generate(maze.size, (col) {
                return _buildCell(row, col, cellSize);
              }),
            );
          }),
        ),
      ),
    );
  }

  Widget _buildCell(int row, int col, double size) {
    final isObstacle = maze.obstacles.any((o) => o[0] == col && o[1] == row);
    final isExit = maze.exits.any((e) => e[0] == col && e[1] == row);
    
    // Check if scout robot is at this position
    final robot = robots.firstWhere(
      (r) => r.position[0] == col && r.position[1] == row,
      orElse: () => RobotData(
        id: '',
        position: [-1, -1],
        status: '',
        battery: 0,
        exploredArea: [],
      ),
    );
    final hasRobot = robot.position[0] != -1;
    
    // Check if human is at this position
    final human = humans.firstWhere(
      (h) => h.position[0] == col && h.position[1] == row,
      orElse: () => HumanData(
        id: '',
        position: [-1, -1],
        evacuationPath: [],
        priority: '',
        distanceToExit: 0,
        exitTarget: [0, 0],
      ),
    );
    final hasHuman = human.position[0] != -1;

    // Check if this cell is on any evacuation path
    final isOnEvacuationPath = humans.any((h) => 
      h.evacuationPath.any((p) => p[0] == col && p[1] == row)
    );

    // Check if this cell is in explored area
    final isExplored = robots.any((r) => 
      r.exploredArea.any((p) => p[0] == col && p[1] == row)
    );

    Color bgColor = Colors.white;
    if (isObstacle) {
      bgColor = Colors.grey.shade800;
    } else if (isExit) {
      bgColor = Colors.green.shade100;
    } else if (isOnEvacuationPath) {
      bgColor = Colors.blue.shade50; // Evacuation path
    } else if (isExplored) {
      bgColor = Colors.grey.shade100; // Explored by scouts
    }

    return Container(
      width: size,
      height: size,
      decoration: BoxDecoration(
        color: bgColor,
        border: Border.all(color: Colors.grey.shade400, width: 0.5),
      ),
      child: Stack(
        children: [
          if (isExit)
            Center(
              child: Icon(
                Icons.exit_to_app, 
                color: Colors.green.shade700, 
                size: size * 0.5,
              ),
            ),
          if (hasRobot)
            Center(
              child: Container(
                width: size * 0.7,
                height: size * 0.7,
                decoration: BoxDecoration(
                  color: robot.status == 'exploring' || robot.status == 'mapping'
                    ? Colors.purple.shade600
                    : Colors.grey.shade600,
                  shape: BoxShape.circle,
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.2),
                      blurRadius: 4,
                      offset: const Offset(0, 2),
                    ),
                  ],
                ),
                child: Icon(
                  Icons.explore,
                  color: Colors.white,
                  size: size * 0.4,
                ),
              ),
            ),
          if (hasHuman)
            Center(
              child: Container(
                width: size * 0.65,
                height: size * 0.65,
                decoration: BoxDecoration(
                  color: human.priority == 'high'
                      ? Colors.red.shade600
                      : human.priority == 'medium'
                          ? Colors.orange.shade600
                          : Colors.yellow.shade800,
                  shape: BoxShape.circle,
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.2),
                      blurRadius: 4,
                      offset: const Offset(0, 2),
                    ),
                  ],
                ),
                child: Icon(
                  Icons.person,
                  color: Colors.white,
                  size: size * 0.4,
                ),
              ),
            ),
        ],
      ),
    );
  }
}

