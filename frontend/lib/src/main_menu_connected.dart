import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'models/evacuation_state.dart';
import 'services/api_service.dart';
import 'widgets/maze_grid.dart';
import 'login.dart';

class MainMenu extends StatefulWidget {
  const MainMenu({super.key});

  @override
  State<MainMenu> createState() => _MainMenuState();
}

class _MainMenuState extends State<MainMenu> {
  int _selectedIndex = 0;

  final List<Widget> _pages = const [
    DashboardScreen(),
    MapScreen(),
    RobotsScreen(),
    SettingsScreen(),
  ];

  void _selectIndex(int index) {
    setState(() => _selectedIndex = index);
    Navigator.of(context).maybePop();
  }

  Future<void> _refreshData(BuildContext context) async {
    final state = Provider.of<EvacuationState>(context, listen: false);
    state.setLoading(true);
    
    try {
      final data = await ApiService().getFlutterUpdate();
      state.updateFromFlutterData(data);
    } catch (e) {
      state.setError('Failed to refresh: $e');
    } finally {
      state.setLoading(false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('ZPM-TUNA'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () => _refreshData(context),
            tooltip: 'Refresh',
          ),
        ],
      ),
      drawer: Drawer(
        child: SafeArea(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const DrawerHeader(
                decoration: BoxDecoration(color: Colors.red),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: [
                    Text(
                      'ZPM-TUNA',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    SizedBox(height: 4),
                    Text(
                      'Zero Panic in Movement',
                      style: TextStyle(color: Colors.white70, fontSize: 12),
                    ),
                  ],
                ),
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
                leading: const Icon(Icons.logout),
                title: const Text('Sign out'),
                onTap: () {
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
          BottomNavigationBarItem(icon: Icon(Icons.settings), label: 'Settings'),
        ],
        type: BottomNavigationBarType.fixed,
      ),
    );
  }
}

// ============================================================================
// DASHBOARD SCREEN - Shows overview with real backend data
// ============================================================================
class DashboardScreen extends StatefulWidget {
  const DashboardScreen({super.key});

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  @override
  void initState() {
    super.initState();
    _loadData();
  }

  Future<void> _loadData() async {
    final state = Provider.of<EvacuationState>(context, listen: false);
    state.setLoading(true);
    
    try {
      final data = await ApiService().getFlutterUpdate();
      state.updateFromFlutterData(data);
    } catch (e) {
      state.setError('Failed to load data: $e');
    } finally {
      state.setLoading(false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<EvacuationState>(
      builder: (context, state, child) {
        if (state.isLoading) {
          return const Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                CircularProgressIndicator(),
                SizedBox(height: 16),
                Text('Connecting to backend...'),
              ],
            ),
          );
        }

        if (state.error != null) {
          return Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Icon(Icons.error_outline, size: 48, color: Colors.red),
                const SizedBox(height: 16),
                Text(state.error!),
                const SizedBox(height: 8),
                const Text('Backend might not be running'),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: _loadData,
                  child: const Text('Retry Connection'),
                ),
              ],
            ),
          );
        }

        return RefreshIndicator(
          onRefresh: _loadData,
          child: SingleChildScrollView(
            physics: const AlwaysScrollableScrollPhysics(),
            padding: EdgeInsets.symmetric(
              horizontal: MediaQuery.of(context).size.width > 600 ? 24 : 16,
              vertical: 16,
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                // Status Cards
                Row(
                  children: [
                    Expanded(
                      child: _StatusCard(
                        title: 'Scout Robots',
                        value: '${state.robots.length}',
                        icon: Icons.explore,
                        color: Colors.purple,
                      ),
                    ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: _StatusCard(
                        title: 'People',
                        value: '${state.humans.length}',
                        icon: Icons.person,
                        color: Colors.orange,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 12),
                Row(
                  children: [
                    Expanded(
                      child: _StatusCard(
                        title: 'Exits',
                        value: '${state.maze?.exits.length ?? 0}',
                        icon: Icons.exit_to_app,
                        color: Colors.green,
                      ),
                    ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: _StatusCard(
                        title: 'Obstacles',
                        value: '${state.maze?.obstacles.length ?? 0}',
                        icon: Icons.block,
                        color: Colors.grey,
                      ),
                    ),
                  ],
                ),
                
                // AI Guidance
                if (state.guidance != null && state.guidance!.text.isNotEmpty) ...[
                  const SizedBox(height: 20),
                  Card(
                    elevation: 4,
                    child: Container(
                      padding: const EdgeInsets.all(20),
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(16),
                        gradient: LinearGradient(
                          begin: Alignment.topLeft,
                          end: Alignment.bottomRight,
                          colors: [
                            Colors.blue.shade50,
                            Colors.white,
                          ],
                        ),
                      ),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Row(
                            children: [
                              Container(
                                padding: const EdgeInsets.all(10),
                                decoration: BoxDecoration(
                                  color: Colors.blue.shade100,
                                  borderRadius: BorderRadius.circular(12),
                                ),
                                child: Icon(
                                  Icons.navigation,
                                  color: Colors.blue.shade700,
                                  size: 24,
                                ),
                              ),
                              const SizedBox(width: 12),
                              const Expanded(
                                child: Text(
                                  'Evacuation Instructions',
                                  style: TextStyle(
                                    fontSize: 18,
                                    fontWeight: FontWeight.bold,
                                  ),
                                ),
                              ),
                            ],
                          ),
                          const SizedBox(height: 16),
                          Container(
                            padding: const EdgeInsets.all(16),
                            decoration: BoxDecoration(
                              color: Colors.white,
                              borderRadius: BorderRadius.circular(12),
                              border: Border.all(
                                color: Colors.blue.shade200,
                                width: 2,
                              ),
                            ),
                            child: Text(
                              state.guidance!.text,
                              style: const TextStyle(
                                fontSize: 15,
                                height: 1.5,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
                
                const SizedBox(height: 20),
                
                // Connection Status Indicator
                Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.green.shade50,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.green.shade200),
                  ),
                  child: Row(
                    children: [
                      Icon(Icons.check_circle, color: Colors.green.shade600, size: 20),
                      const SizedBox(width: 8),
                      Text(
                        'Connected to Backend',
                        style: TextStyle(
                          color: Colors.green.shade700,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}

class _StatusCard extends StatelessWidget {
  final String title;
  final String value;
  final IconData icon;
  final Color color;

  const _StatusCard({
    required this.title,
    required this.value,
    required this.icon,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    final isSmallScreen = MediaQuery.of(context).size.width < 600;
    
    return Card(
      elevation: 3,
      child: Container(
        padding: EdgeInsets.all(isSmallScreen ? 16 : 20),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(16),
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              color.withOpacity(0.1),
              color.withOpacity(0.05),
            ],
          ),
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              padding: EdgeInsets.all(isSmallScreen ? 10 : 12),
              decoration: BoxDecoration(
                color: color.withOpacity(0.2),
                shape: BoxShape.circle,
              ),
              child: Icon(icon, size: isSmallScreen ? 28 : 32, color: color),
            ),
            SizedBox(height: isSmallScreen ? 8 : 12),
            Text(
              value,
              style: TextStyle(
                fontSize: isSmallScreen ? 24 : 28,
                fontWeight: FontWeight.bold,
                color: Colors.grey.shade800,
              ),
            ),
            const SizedBox(height: 4),
            Text(
              title,
              textAlign: TextAlign.center,
              maxLines: 2,
              overflow: TextOverflow.ellipsis,
              style: TextStyle(
                fontSize: isSmallScreen ? 11 : 13,
                color: Colors.grey.shade600,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// ============================================================================
// MAP SCREEN - Shows maze grid with real data
// ============================================================================
class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  @override
  void initState() {
    super.initState();
    _loadData();
  }

  Future<void> _loadData() async {
    final state = Provider.of<EvacuationState>(context, listen: false);
    if (state.maze == null) {
      state.setLoading(true);
      try {
        final data = await ApiService().getFlutterUpdate();
        state.updateFromFlutterData(data);
      } catch (e) {
        state.setError('Failed to load maze: $e');
      } finally {
        state.setLoading(false);
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<EvacuationState>(
      builder: (context, state, child) {
        if (state.isLoading) {
          return const Center(child: CircularProgressIndicator());
        }

        if (state.error != null || state.maze == null) {
          return Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Icon(Icons.error_outline, size: 48, color: Colors.red),
                const SizedBox(height: 16),
                Text(state.error ?? 'No maze data available'),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: _loadData,
                  child: const Text('Retry'),
                ),
              ],
            ),
          );
        }

        return RefreshIndicator(
          onRefresh: _loadData,
          child: SingleChildScrollView(
            physics: const AlwaysScrollableScrollPhysics(),
            padding: const EdgeInsets.all(20),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                Text(
                  'Live Maze View',
                  style: Theme.of(context).textTheme.headlineSmall?.copyWith(
                    fontWeight: FontWeight.bold,
                  ),
                ),
                const SizedBox(height: 8),
                Text(
                  '${state.maze!.size}x${state.maze!.size} Grid',
                  style: TextStyle(color: Colors.grey.shade600),
                ),
                const SizedBox(height: 24),
                
                // Maze Grid
                Center(
                  child: MazeGrid(
                    maze: state.maze!,
                    robots: state.robots,
                    humans: state.humans,
                  ),
                ),
                
                const SizedBox(height: 24),
                
                // Legend
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text(
                          'Legend',
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                        const SizedBox(height: 12),
                        _LegendItem(
                          icon: Icons.explore,
                          color: Colors.purple,
                          label: 'Scout Robot',
                        ),
                        _LegendItem(
                          icon: Icons.person,
                          color: Colors.red,
                          label: 'Person (High Priority)',
                        ),
                        _LegendItem(
                          icon: Icons.exit_to_app,
                          color: Colors.green,
                          label: 'Exit',
                        ),
                        _LegendItem(
                          icon: Icons.block,
                          color: Colors.grey.shade800,
                          label: 'Obstacle',
                        ),
                        _LegendItem(
                          icon: Icons.route,
                          color: Colors.blue.shade200,
                          label: 'Evacuation Path',
                        ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}

class _LegendItem extends StatelessWidget {
  final IconData icon;
  final Color color;
  final String label;

  const _LegendItem({
    required this.icon,
    required this.color,
    required this.label,
  });

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Icon(icon, color: color, size: 20),
          const SizedBox(width: 12),
          Text(label),
        ],
      ),
    );
  }
}

// ============================================================================
// ROBOTS SCREEN - Shows robot list with real data
// ============================================================================
class RobotsScreen extends StatefulWidget {
  const RobotsScreen({super.key});

  @override
  State<RobotsScreen> createState() => _RobotsScreenState();
}

class _RobotsScreenState extends State<RobotsScreen> {
  @override
  void initState() {
    super.initState();
    _loadData();
  }

  Future<void> _loadData() async {
    final state = Provider.of<EvacuationState>(context, listen: false);
    state.setLoading(true);
    try {
      final data = await ApiService().getFlutterUpdate();
      state.updateFromFlutterData(data);
    } catch (e) {
      state.setError('Failed to load robots: $e');
    } finally {
      state.setLoading(false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<EvacuationState>(
      builder: (context, state, child) {
        if (state.isLoading) {
          return const Center(child: CircularProgressIndicator());
        }

        if (state.error != null) {
          return Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Icon(Icons.error_outline, size: 48, color: Colors.red),
                const SizedBox(height: 16),
                Text(state.error!),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: _loadData,
                  child: const Text('Retry'),
                ),
              ],
            ),
          );
        }

        if (state.robots.isEmpty) {
          return const Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.explore_outlined, size: 64, color: Colors.grey),
                SizedBox(height: 16),
                Text('No scout robots active'),
              ],
            ),
          );
        }

        return RefreshIndicator(
          onRefresh: _loadData,
          child: ListView.builder(
            padding: const EdgeInsets.all(16),
            itemCount: state.robots.length,
            itemBuilder: (context, index) {
              final robot = state.robots[index];
              return Card(
                margin: const EdgeInsets.only(bottom: 12),
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          CircleAvatar(
                            backgroundColor: Colors.purple,
                            child: const Icon(Icons.explore, color: Colors.white),
                          ),
                          const SizedBox(width: 12),
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  robot.id.toUpperCase(),
                                  style: const TextStyle(
                                    fontSize: 18,
                                    fontWeight: FontWeight.bold,
                                  ),
                                ),
                                Text(
                                  robot.status.toUpperCase(),
                                  style: TextStyle(
                                    color: Colors.purple.shade600,
                                    fontWeight: FontWeight.w500,
                                  ),
                                ),
                              ],
                            ),
                          ),
                        ],
                      ),
                      const Divider(height: 24),
                      _InfoRow(
                        icon: Icons.location_on,
                        label: 'Position',
                        value: '[${robot.position[0]}, ${robot.position[1]}]',
                      ),
                      _InfoRow(
                        icon: Icons.battery_charging_full,
                        label: 'Battery',
                        value: '${robot.battery}%',
                      ),
                      _InfoRow(
                        icon: Icons.explore,
                        label: 'Explored Area',
                        value: '${robot.exploredArea.length} cells',
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        );
      },
    );
  }
}

class _InfoRow extends StatelessWidget {
  final IconData icon;
  final String label;
  final String value;

  const _InfoRow({
    required this.icon,
    required this.label,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Icon(icon, size: 18, color: Colors.grey.shade600),
          const SizedBox(width: 8),
          Text(
            '$label: ',
            style: TextStyle(color: Colors.grey.shade600),
          ),
          Text(
            value,
            style: const TextStyle(fontWeight: FontWeight.w500),
          ),
        ],
      ),
    );
  }
}

// ============================================================================
// SETTINGS SCREEN - System configuration
// ============================================================================
class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  String _selectedMode = 'auto';
  bool _voiceGuidance = true;
  bool _autoRefresh = true;
  int _refreshInterval = 5;
  bool _showPaths = true;
  bool _showExploredAreas = true;

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        const Text(
          'System Settings',
          style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 24),
        
        // Operation Mode
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Operation Mode',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 16),
                RadioListTile<String>(
                  title: const Text('Automatic'),
                  subtitle: const Text('AI-guided evacuation with full automation'),
                  value: 'auto',
                  groupValue: _selectedMode,
                  onChanged: (value) => setState(() => _selectedMode = value!),
                ),
                RadioListTile<String>(
                  title: const Text('Manual'),
                  subtitle: const Text('Manual robot control and path planning'),
                  value: 'manual',
                  groupValue: _selectedMode,
                  onChanged: (value) => setState(() => _selectedMode = value!),
                ),
                RadioListTile<String>(
                  title: const Text('Demo'),
                  subtitle: const Text('Demonstration mode with mock data'),
                  value: 'demo',
                  groupValue: _selectedMode,
                  onChanged: (value) => setState(() => _selectedMode = value!),
                ),
              ],
            ),
          ),
        ),
        
        const SizedBox(height: 16),
        
        // Guidance Options
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Guidance Options',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 8),
                SwitchListTile(
                  title: const Text('Voice Guidance'),
                  subtitle: const Text('Enable text-to-speech for evacuation instructions'),
                  value: _voiceGuidance,
                  onChanged: (value) => setState(() => _voiceGuidance = value),
                ),
                SwitchListTile(
                  title: const Text('Show Evacuation Paths'),
                  subtitle: const Text('Display calculated paths on the maze'),
                  value: _showPaths,
                  onChanged: (value) => setState(() => _showPaths = value),
                ),
                SwitchListTile(
                  title: const Text('Show Explored Areas'),
                  subtitle: const Text('Highlight areas mapped by scout robots'),
                  value: _showExploredAreas,
                  onChanged: (value) => setState(() => _showExploredAreas = value),
                ),
              ],
            ),
          ),
        ),
        
        const SizedBox(height: 16),
        
        // Data Refresh
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Data Refresh',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 8),
                SwitchListTile(
                  title: const Text('Auto Refresh'),
                  subtitle: const Text('Automatically fetch updates from backend'),
                  value: _autoRefresh,
                  onChanged: (value) => setState(() => _autoRefresh = value),
                ),
                if (_autoRefresh) ...[
                  const SizedBox(height: 8),
                  ListTile(
                    title: const Text('Refresh Interval'),
                    subtitle: Slider(
                      value: _refreshInterval.toDouble(),
                      min: 1,
                      max: 30,
                      divisions: 29,
                      label: '$_refreshInterval seconds',
                      onChanged: (value) => setState(() => _refreshInterval = value.toInt()),
                    ),
                    trailing: Text('${_refreshInterval}s'),
                  ),
                ],
              ],
            ),
          ),
        ),
        
        const SizedBox(height: 16),
        
        // Backend Connection
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Backend Connection',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 16),
                ListTile(
                  leading: const Icon(Icons.link),
                  title: const Text('Backend URL'),
                  subtitle: Text(ApiService.baseUrl),
                ),
                ListTile(
                  leading: const Icon(Icons.data_usage),
                  title: const Text('Mock Data'),
                  subtitle: Text(ApiService.useMockData ? 'Enabled' : 'Disabled'),
                  trailing: Icon(
                    ApiService.useMockData ? Icons.warning : Icons.check_circle,
                    color: ApiService.useMockData ? Colors.orange : Colors.green,
                  ),
                ),
                const SizedBox(height: 8),
                ElevatedButton.icon(
                  onPressed: () async {
                    try {
                      final health = await ApiService().healthCheck();
                      if (context.mounted) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Text('Backend is ${health['status']}'),
                            backgroundColor: Colors.green,
                          ),
                        );
                      }
                    } catch (e) {
                      if (context.mounted) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Text('Backend connection failed: $e'),
                            backgroundColor: Colors.red,
                          ),
                        );
                      }
                    }
                  },
                  icon: const Icon(Icons.network_check),
                  label: const Text('Test Connection'),
                ),
              ],
            ),
          ),
        ),
        
        const SizedBox(height: 16),
        
        // Actions
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                const Text(
                  'Actions',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 16),
                ElevatedButton.icon(
                  onPressed: () async {
                    try {
                      await ApiService().setupDemo();
                      if (context.mounted) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            content: Text('Demo scenario initialized'),
                            backgroundColor: Colors.green,
                          ),
                        );
                      }
                    } catch (e) {
                      if (context.mounted) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Text('Failed to setup demo: $e'),
                            backgroundColor: Colors.red,
                          ),
                        );
                      }
                    }
                  },
                  icon: const Icon(Icons.play_arrow),
                  label: const Text('Setup Demo Scenario'),
                ),
                const SizedBox(height: 8),
                OutlinedButton.icon(
                  onPressed: () {
                    // Reset to defaults
                    setState(() {
                      _selectedMode = 'auto';
                      _voiceGuidance = true;
                      _autoRefresh = true;
                      _refreshInterval = 5;
                      _showPaths = true;
                      _showExploredAreas = true;
                    });
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('Settings reset to defaults')),
                    );
                  },
                  icon: const Icon(Icons.restore),
                  label: const Text('Reset to Defaults'),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

