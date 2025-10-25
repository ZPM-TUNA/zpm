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
class DashboardScreen extends StatefulWidget {
  const DashboardScreen({super.key});

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  List<Map<String, Object>> _alerts = [];

  @override
  void initState() {
    super.initState();
    _generateAlerts();
  }

  void _generateAlerts() {
    // Sample robot state (mirror of RobotsScreen's mock data).
    final robots = [
      {'id': 'r1', 'name': 'EVA-1', 'battery': 92, 'online': true},
      {'id': 'r2', 'name': 'Scout-2', 'battery': 58, 'online': true},
      {'id': 'r3', 'name': 'Helper-3', 'battery': 14, 'online': false},
    ];

    final now = DateTime.now();
    final List<Map<String, Object>> alerts = [];

    for (final r in robots) {
      final name = r['name'] as String;
      final battery = r['battery'] as int;
      final online = r['online'] as bool;

      if (!online) {
        alerts.add({
          'title': '$name is offline',
          'severity': 'high',
          'time': now.subtract(const Duration(minutes: 5)),
          'tab': 2,
        });
      }

      if (battery < 20) {
        alerts.add({
          'title': '$name battery critically low (${battery}%)',
          'severity': 'high',
          'time': now.subtract(const Duration(minutes: 10)),
          'tab': 2,
        });
      } else if (battery < 40) {
        alerts.add({
          'title': '$name battery low (${battery}%)',
          'severity': 'medium',
          'time': now.subtract(const Duration(minutes: 15)),
          'tab': 2,
        });
      }
    }

    // Map-related sample alert.
    alerts.add({
      'title': 'Robots detected multiple people in Zone C',
      'severity': 'high',
      'time': now.subtract(const Duration(minutes: 1)),
      'tab': 1,
    });

    setState(() => _alerts = alerts);
  }

  Color _colorFor(String severity) {
    switch (severity) {
      case 'high':
        return Colors.red;
      case 'medium':
        return Colors.orange;
      default:
        return Colors.blue;
    }
  }

  IconData _iconFor(String severity) {
    switch (severity) {
      case 'high':
        return Icons.error_outline;
      case 'medium':
        return Icons.warning_amber_outlined;
      default:
        return Icons.info_outline;
    }
  }

  String _relativeTime(DateTime t) {
    final diff = DateTime.now().difference(t);
    if (diff.inMinutes < 1) return 'just now';
    if (diff.inMinutes < 60) return '${diff.inMinutes}m ago';
    if (diff.inHours < 24) return '${diff.inHours}h ago';
    return '${diff.inDays}d ago';
  }

  @override
  Widget build(BuildContext context) {
    final menuState = context.findAncestorStateOfType<_MainMenuState>();

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Padding(
          padding: const EdgeInsets.all(12.0),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              const Text('Alerts', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
              Row(children: [
                IconButton(
                  onPressed: _generateAlerts,
                  icon: const Icon(Icons.refresh),
                ),
              ])
            ],
          ),
        ),
        Expanded(
          child: _alerts.isEmpty
              ? const Center(child: Text('No active alerts'))
              : ListView.builder(
                  padding: const EdgeInsets.only(bottom: 12),
                  itemCount: _alerts.length,
                  itemBuilder: (context, index) {
                    final a = _alerts[index];
                    final severity = a['severity'] as String;
                    final time = a['time'] as DateTime;
                    final tab = a['tab'] as int;
                    return Card(
                      margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                      child: ListTile(
                        leading: CircleAvatar(
                          backgroundColor: _colorFor(severity).withOpacity(0.15),
                          child: Icon(_iconFor(severity), color: _colorFor(severity)),
                        ),
                        title: Text(a['title'] as String),
                        subtitle: Text(_relativeTime(time)),
                        trailing: ElevatedButton(
                          onPressed: () {
                            // switch to the appropriate tab in the main menu
                            if (menuState != null) menuState._selectIndex(tab);
                          },
                          child: const Text('View'),
                        ),
                      ),
                    );
                  },
                ),
        ),
      ],
    );
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
