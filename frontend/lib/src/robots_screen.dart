import 'package:flutter/material.dart';

class Robot {
  Robot({required this.id, required this.name, required this.battery, required this.online});

  final String id;
  final String name;
  int battery; // 0-100
  bool online;
}

class RobotsScreen extends StatefulWidget {
  const RobotsScreen({super.key});

  @override
  State<RobotsScreen> createState() => _RobotsScreenState();
}

class _RobotsScreenState extends State<RobotsScreen> {
  final List<Robot> _robots = [];

  @override
  void initState() {
    super.initState();
    // Sample/mock data; replace with real data fetching later.
    _robots.addAll([
      Robot(id: 'r1', name: 'EVA-1', battery: 92, online: true),
      Robot(id: 'r2', name: 'Scout-2', battery: 58, online: true),
      Robot(id: 'r3', name: 'Helper-3', battery: 14, online: false),
    ]);
  }



  Widget _buildRobotTile(Robot robot) {
  final batteryColor = robot.battery > 60
    ? Colors.green
    : (robot.battery > 30 ? Colors.orange : Colors.red);

  // Slightly more visible background for the empty part of the battery bar.
  final backgroundBarColor = Theme.of(context).brightness == Brightness.dark
    ? Colors.grey.shade700
    : Colors.grey.shade300;

    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      child: ListTile(
        leading: CircleAvatar(
          backgroundColor: robot.online ? Colors.green[100] : Colors.grey[300],
          child: Icon(robot.online ? Icons.smart_toy : Icons.power_off, color: robot.online ? Colors.green[800] : Colors.grey[700]),
        ),
        title: Text(robot.name),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const SizedBox(height: 6),
            ClipRRect(
              borderRadius: BorderRadius.circular(8),
              child: LinearProgressIndicator(
                value: robot.battery / 100.0,
                color: batteryColor,
                backgroundColor: backgroundBarColor,
                minHeight: 10,
              ),
            ),
            const SizedBox(height: 6),
            Text('Battery: ${robot.battery}%', style: TextStyle(color: batteryColor)),
          ],
        ),
        trailing: Text(
          robot.online ? 'On' : 'Off',
          style: TextStyle(
            color: robot.online ? Colors.green : Colors.red,
            fontWeight: FontWeight.bold,
          ),
        ),
        onTap: () {
          // Expand to details or open controls later
          ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('${robot.name} selected')));
        },
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return _robots.isEmpty
        ? const Center(child: Text('No robots connected'))
        : ListView.builder(
            padding: const EdgeInsets.only(top: 12, bottom: 12),
            itemCount: _robots.length,
            itemBuilder: (context, index) => _buildRobotTile(_robots[index]),
          );
  }
}
