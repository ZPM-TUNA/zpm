import 'package:flutter/material.dart';
import 'main_menu.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  final TextEditingController _emailController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  bool _loading = false;
  final _formKey = GlobalKey<FormState>();
  bool _submitted = false;

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
    setState(() {
      _loading = true;
      _submitted = true;
    });

    // validate form
    final isValid = _formKey.currentState?.validate() ?? false;
    if (!isValid) {
      setState(() => _loading = false);
      _showMessage('Please enter your email and password');
      return;
    }

    await Future.delayed(const Duration(milliseconds: 600)); // simulate work
    setState(() => _loading = false);

    final email = _emailController.text.trim();
    // TODO: Replace with real authentication.
    _showMessage('Signed in as $email (demo)');
    if (!mounted) return;
    Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => const MainMenu()));
  }

  Future<void> _handleGoogleSignIn() async {
    setState(() => _loading = true);
    // Simulate Google sign-in flow. Replace with firebase_auth or google_sign_in later.
    await Future.delayed(const Duration(milliseconds: 900));
    setState(() => _loading = false);

    _showMessage('Signed in with Google (demo)');
    if (!mounted) return;
    Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => const MainMenu()));
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
                  'ZeroPanic — Find safe exits quickly. Sign in to access maps, robots and evacuation tools.',
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 20),

                Form(
                  key: _formKey,
                  child: Column(
                    children: [
                      TextFormField(
                        controller: _emailController,
                        keyboardType: TextInputType.emailAddress,
                        autovalidateMode: _submitted ? AutovalidateMode.always : AutovalidateMode.disabled,
                        decoration: InputDecoration(
                          labelText: 'Email',
                          border: OutlineInputBorder(borderRadius: BorderRadius.circular(16.0)),
                          prefixIcon: const Icon(Icons.email),
                        ),
                        validator: (value) {
                          if (value == null || value.trim().isEmpty) return 'Please enter your email';
                          return null;
                        },
                      ),
                      const SizedBox(height: 12),
                      TextFormField(
                        controller: _passwordController,
                        obscureText: true,
                        autovalidateMode: _submitted ? AutovalidateMode.always : AutovalidateMode.disabled,
                        decoration: InputDecoration(
                          labelText: 'Password',
                          border: OutlineInputBorder(borderRadius: BorderRadius.circular(16.0)),
                          prefixIcon: const Icon(Icons.lock),
                        ),
                        validator: (value) {
                          if (value == null || value.isEmpty) return 'Please enter your password';
                          return null;
                        },
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 16),

                ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color.fromARGB(255, 255, 112, 67), // slightly brighter orange-red
                    foregroundColor: Colors.white,
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16.0)),
                    padding: const EdgeInsets.symmetric(vertical: 14.0),
                  ),
                  onPressed: _loading ? null : _handleSignIn,
                  child: _loading
                      ? const SizedBox(height: 20, width: 20, child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white))
                      : const Text('Sign in'),
                ),

                const SizedBox(height: 12),
                Row(children: const [Expanded(child: Divider()), Padding(padding: EdgeInsets.symmetric(horizontal:8.0), child: Text('or')), Expanded(child: Divider())]),
                const SizedBox(height: 12),

                ElevatedButton.icon(
                  style: ElevatedButton.styleFrom(backgroundColor: Colors.white, foregroundColor: Colors.black),
                  onPressed: _loading ? null : _handleGoogleSignIn,
                  icon: Semantics(
                    label: 'Google logo',
                    image: true,
                    child: Image.network(
                      'https://developers.google.com/identity/images/g-logo.png',
                      height: 20,
                      width: 20,
                      errorBuilder: (context, error, stack) => const Icon(Icons.login),
                    ),
                  ),
                  label: const Text('Sign in with Google'),
                ),

                const SizedBox(height: 18),
                TextButton(
                  onPressed: () => _showMessage('Forgot password flow (not implemented)'),
                  child: const Text('Forgot password?'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
