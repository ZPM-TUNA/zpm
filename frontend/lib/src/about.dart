import 'package:flutter/material.dart';

class AboutScreen extends StatelessWidget {
  const AboutScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: const Text('About'),
      ),
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(16),
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 800),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Center(
                  child: Column(
                    children: [
                      CircleAvatar(
                        radius: 36,
                        backgroundColor: theme.colorScheme.primary,
                        child: Icon(Icons.shield, color: theme.colorScheme.onPrimary, size: 36),
                      ),
                      const SizedBox(height: 12),
                      Text('ZeroPanic', style: theme.textTheme.headlineSmall?.copyWith(fontWeight: FontWeight.bold)),
                      const SizedBox(height: 6),
                      Text('Evacuation coordination & robot assistant', style: theme.textTheme.bodyMedium),
                    ],
                  ),
                ),

                const SizedBox(height: 20),

                Text('What is this?', style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600)),
                const SizedBox(height: 8),
                const Text(
                  'ZeroPanic is a prototype mobile application created to coordinate robot-assisted building evacuations. '
                  'It collects robot status, shows a simplified floor map, and provides a dashboard of safety alerts so responders can act quickly.',
                ),

                const SizedBox(height: 16),
                Text('Key features', style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600)),
                const SizedBox(height: 8),
                const BulletList(items: [
                  'Live robot status (battery, online/offline)',
                  'Dashboard alerts and quick actions',
                  'Map view for robot positions and floor plans (placeholder)',
                  'Settings with theme and account management',
                ]),

                const SizedBox(height: 16),
                Text('For developers', style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600)),
                const SizedBox(height: 8),
                const Text(
                  'This project is structured as a Flutter frontend with a small Python backend (under `backend/`) that handles robot coordination and pathfinding. ' 
                  'The UI uses Provider for state management and is intentionally modular to make it easy to extend.',
                ),

                const SizedBox(height: 16),
                Text('Credits', style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600)),
                const SizedBox(height: 8),
                const Text('Created by Aashish Anantharaman, Madhav Khanal, George Zhao, and Jason Sacerio.'),

                const SizedBox(height: 20),
                Center(
                  child: Text('Version: 0.1.0', style: theme.textTheme.bodySmall),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class BulletList extends StatelessWidget {
  const BulletList({super.key, required this.items});
  final List<String> items;

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: items
          .map((t) => Padding(
                padding: const EdgeInsets.symmetric(vertical: 4.0),
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('• ', style: TextStyle(fontSize: 18)),
                    Expanded(child: Text(t)),
                  ],
                ),
              ))
          .toList(),
    );
  }
}