import 'package:flutter/material.dart';

class ZeroPanicApp extends StatelessWidget {
  const ZeroPanicApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
  title: 'ZeroPanic',
      theme: ThemeData(primarySwatch: Colors.red, useMaterial3: true),
      home: const LoginScreen(),
    );
  }
}

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  final TextEditingController _emailController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  bool _loading = false;

  @override
  void dispose() {
    _emailController.dispose();
    _passwordController.dispose();
    super.dispose();
  }

  void _showMessage(String message) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(message)));
  }

  Future<void> _handleSignIn() async {
    setState(() => _loading = true);
    await Future.delayed(const Duration(milliseconds: 600)); // simulate work
    setState(() => _loading = false);

    final email = _emailController.text.trim();
    if (email.isEmpty || _passwordController.text.isEmpty) {
      _showMessage('Enter email and password');
      return;
    }

    // Placeholder: authenticate here. For now show a message and go to Home.
    _showMessage('Signed in as $email (demo)');
    if (!mounted) return;
    Navigator.of(context).push(MaterialPageRoute(builder: (_) => const HomeScreen()));
  }

  void _continueAsGuest() {
    _showMessage('Continuing as guest (limited)');
    if (!mounted) return;
    Navigator.of(context).push(MaterialPageRoute(builder: (_) => const HomeScreen()));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
  appBar: AppBar(title: const Text('ZeroPanic')),
      body: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(24.0),
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 480),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                const SizedBox(height: 8),
                const Text(
                  'Find the safest exit during a fire. Robots will help locate humans and guide you.',
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 24),

                TextField(
                  controller: _emailController,
                  keyboardType: TextInputType.emailAddress,
                  decoration: const InputDecoration(
                    labelText: 'Email',
                    border: OutlineInputBorder(),
                  ),
                ),
                const SizedBox(height: 12),
                TextField(
                  controller: _passwordController,
                  obscureText: true,
                  decoration: const InputDecoration(
                    labelText: 'Password',
                    border: OutlineInputBorder(),
                  ),
                ),
                const SizedBox(height: 18),

                ElevatedButton(
                  onPressed: _loading ? null : _handleSignIn,
                  child: _loading
                      ? const SizedBox(
                          height: 20,
                          width: 20,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Text('Sign in'),
                ),

                const SizedBox(height: 12),
                ElevatedButton.icon(
                  onPressed: () => _showMessage('Social sign-in (placeholder)'),
                  icon: const Icon(Icons.login),
                  label: const Text('Sign in with Google'),
                ),

                const SizedBox(height: 12),
                TextButton(
                  onPressed: _continueAsGuest,
                  child: const Text('Continue as Guest (limited)'),
                ),

                const SizedBox(height: 24),
                OutlinedButton(
                  onPressed: () => _showMessage('Demo: Open map / pathfinding (not implemented)'),
                  child: const Text('Start (Find Exit)'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class HomeScreen extends StatelessWidget {
  const HomeScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
  appBar: AppBar(title: const Text('ZeroPanic — Home')),
      body: const Center(child: Text('Home screen placeholder — map and robot features go here')),
    );
  }
}
