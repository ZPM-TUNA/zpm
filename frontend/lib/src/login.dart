import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'main_menu.dart';
import 'models/auth_provider.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  bool _isInitialized = false;

  @override
  void initState() {
    super.initState();
    _initializeAuth();
  }

  Future<void> _initializeAuth() async {
    final authProvider = context.read<AuthProvider>();
    await authProvider.initialize();

    if (mounted) {
      setState(() {
        _isInitialized = true;
      });

      // If user is already signed in, navigate to main menu
      if (authProvider.isSignedIn) {
        Navigator.of(context).pushReplacement(
          MaterialPageRoute(builder: (_) => const MainMenu())
        );
      }
    }
  }

  void _showMessage(String message) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(message)));
  }

  Future<void> _handleGoogleSignIn() async {
    final authProvider = context.read<AuthProvider>();
    final success = await authProvider.signInWithGoogle();

    if (success && mounted) {
      _showMessage('Signed in with Google successfully!');
      Navigator.of(context).pushReplacement(
        MaterialPageRoute(builder: (_) => const MainMenu())
      );
    } else if (mounted) {
      _showMessage('Failed to sign in with Google. Please try again.');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ZeroPanic')),
      body: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.symmetric(horizontal: 24.0, vertical: 28.0),
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 480),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                const SizedBox(height: 8),
                // Match dashboard/about avatar
                Builder(builder: (context) {
                  final theme = Theme.of(context);
                  return Column(
                    children: [
                      CircleAvatar(
                        radius: 36,
                        backgroundColor: const Color(0xFFFF7043), // match Sign in button color
                        child: Icon(Icons.shield, color: theme.colorScheme.onPrimary, size: 36),
                      ),
                      const SizedBox(height: 8),
                      Text('ZeroPanic', style: theme.textTheme.headlineSmall?.copyWith(fontWeight: FontWeight.bold)),
                    ],
                  );
                }),
                const SizedBox(height: 12),
                const Text(
                  'ZeroPanic — Find safe exits quickly. Sign in with Google to access maps, robots and evacuation tools.',
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 40),

                if (!_isInitialized)
                  const Center(
                    child: CircularProgressIndicator(),
                  )
                else
                  Consumer<AuthProvider>(
                    builder: (context, authProvider, _) {
                      return ElevatedButton.icon(
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.white,
                          foregroundColor: Colors.black,
                          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16.0)),
                          padding: const EdgeInsets.symmetric(vertical: 16.0),
                        ),
                        onPressed: authProvider.isLoading ? null : _handleGoogleSignIn,
                        icon: authProvider.isLoading
                            ? const SizedBox(
                                height: 20,
                                width: 20,
                                child: CircularProgressIndicator(strokeWidth: 2, color: Colors.black),
                              )
                            : Semantics(
                                label: 'Google logo',
                                image: true,
                                child: Image.network(
                                  'https://developers.google.com/identity/images/g-logo.png',
                                  height: 20,
                                  width: 20,
                                  errorBuilder: (context, error, stack) => const Icon(Icons.login),
                                ),
                              ),
                        label: Text(
                          authProvider.isLoading ? 'Signing in...' : 'Sign in with Google',
                          style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w500),
                        ),
                      );
                    },
                  ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}