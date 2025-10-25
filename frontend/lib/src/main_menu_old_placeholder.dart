import 'package:flutter/material.dart';
import 'login.dart';

class MainMenu extends StatefulWidget {
  const MainMenu({super.key});

  @override
  State<MainMenu> createState() => _MainMenuState();
}

class _MainMenuState extends State<MainMenu> {
  int _selectedIndex = 0;

  static const List<Widget> _pages = <Widget>[
    DashboardScreen(),
    MapScreen(),
    RobotsScreen(),
  ];

  void _selectIndex(int index) {
    setState(() => _selectedIndex = index);
    Navigator.of(context).maybePop(); // close drawer on selection
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ZeroPanic — Home')),
      drawer: Drawer(
        child: SafeArea(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const DrawerHeader(
                decoration: BoxDecoration(color: Colors.red),
                child: Text('ZeroPanic', style: TextStyle(color: Colors.white, fontSize: 20)),
              ),
              ListTile(
                leading: const Icon(Icons.dashboard),
                title: const Text('Dashboard'),
                selected: _selectedIndex == 0,
                onTap: () => _selectIndex(0),
              ),
              ListTile(
                leading: const Icon(Icons.map),
                title: const Text('Map'),
                selected: _selectedIndex == 1,
                onTap: () => _selectIndex(1),
              ),
              ListTile(
                leading: const Icon(Icons.smart_toy),
                title: const Text('Robots'),
                selected: _selectedIndex == 2,
                onTap: () => _selectIndex(2),
              ),
              ListTile(
                leading: const Icon(Icons.settings),
                title: const Text('Settings'),
                selected: _selectedIndex == 3,
                onTap: () => _selectIndex(3),
              ),
              const Spacer(),
              const Divider(),
              ListTile(
                leading: const Icon(Icons.settings),
                title: const Text('Settings'),
                onTap: () {
                  Navigator.of(context).pop();
                  Navigator.of(context).push(MaterialPageRoute(builder: (_) => const SettingsScreen()));
                },
              ),
              ListTile(
                leading: const Icon(Icons.info_outline),
                title: const Text('About'),
                onTap: () {
                  Navigator.of(context).pop();
                  Navigator.of(context).push(MaterialPageRoute(builder: (_) => const AboutScreen()));
                },
              ),
              ListTile(
                leading: const Icon(Icons.logout),
                title: const Text('Sign out'),
                onTap: () {
                  // Clear navigation stack and go back to LoginScreen
                  Navigator.of(context).pushAndRemoveUntil(
                    MaterialPageRoute(builder: (_) => const LoginScreen()),
                    (route) => false,
                  );
                },
              ),
            ],
          ),
        ),
      ),
      body: IndexedStack(index: _selectedIndex, children: _pages),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _selectedIndex,
        onTap: _selectIndex,
        items: const [
          BottomNavigationBarItem(icon: Icon(Icons.dashboard), label: 'Dashboard'),
          BottomNavigationBarItem(icon: Icon(Icons.map), label: 'Map'),
          BottomNavigationBarItem(icon: Icon(Icons.smart_toy), label: 'Robots'),
        ],
        type: BottomNavigationBarType.fixed,
      ),
    );
  }
}

// Placeholder pages for the main menu sections. Replace with real implementations.
class DashboardScreen extends StatelessWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(child: Padding(
      padding: EdgeInsets.all(16.0),
      child: Text('Dashboard — quick status, alerts, and recent robot activity.'),
    ));
  }
}

class MapScreen extends StatelessWidget {
  const MapScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(child: Text('Map view — floor plan and live robot positions'));
  }
}

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
            LinearProgressIndicator(
              value: robot.battery / 100.0,
              color: batteryColor,
              backgroundColor: Colors.grey.shade200,
              minHeight: 8,
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

class PathfindingScreen extends StatelessWidget {
  const PathfindingScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(child: Text('Pathfinding — run simulation and show suggested exits'));
  }
}

class EvacuationScreen extends StatelessWidget {
  const EvacuationScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(child: Text('Evacuation mode — live guidance and robot assistance'));
  }
}

class SettingsScreen extends StatelessWidget {
  const SettingsScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(child: Text('Settings — preferences and integrations'));
  }
}

class AboutScreen extends StatelessWidget {
  const AboutScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(child: Text('About ZeroPanic — app version and credits'));
  }
}
