import 'package:flutter/material.dart';

class ForgotPasswordScreen extends StatefulWidget {
  const ForgotPasswordScreen({super.key});

  @override
  State<ForgotPasswordScreen> createState() => _ForgotPasswordScreenState();
}

class _ForgotPasswordScreenState extends State<ForgotPasswordScreen> {
  final _formKey = GlobalKey<FormState>();
  final TextEditingController _emailController = TextEditingController();
  bool _isLoading = false;

  @override
  void dispose() {
    _emailController.dispose();
    super.dispose();
  }

  Future<void> _submit() async {
    if (!_formKey.currentState!.validate()) return;

    setState(() => _isLoading = true);
    try {
      // Simulate network call; replace with real API call.
      await Future.delayed(const Duration(seconds: 1));

      // For privacy, show the same message whether the email exists or not.
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('If that email exists, a password reset link has been sent.'),
        ),
      );

      // Optionally navigate back to login after a short delay
      await Future.delayed(const Duration(milliseconds: 600));
      if (mounted) Navigator.of(context).pop();
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  String? _validateEmail(String? v) {
    final s = (v ?? '').trim();
    if (s.isEmpty) return 'Please enter your email';
    final emailReg = RegExp(r"^[^@\s]+@[^@\s]+\.[^@\s]+$");
    if (!emailReg.hasMatch(s)) return 'Enter a valid email address';
    return null;
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Scaffold(
      appBar: AppBar(title: const Text('Forgot Password')),
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const SizedBox(height: 8),
              Text(
                'Enter the email associated with your account and we\'ll send a link to reset your password.',
                style: theme.textTheme.bodyMedium,
              ),
              const SizedBox(height: 20),
              Form(
                key: _formKey,
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    TextFormField(
                      controller: _emailController,
                      keyboardType: TextInputType.emailAddress,
                      autofillHints: const [AutofillHints.email],
                      textInputAction: TextInputAction.done,
                      decoration: const InputDecoration(
                        labelText: 'Email',
                        hintText: 'you@example.com',
                        prefixIcon: Icon(Icons.email_outlined),
                      ),
                      validator: _validateEmail,
                      onFieldSubmitted: (_) => _submit(),
                    ),
                    const SizedBox(height: 18),
                    SizedBox(
                      height: 48,
                      child: ElevatedButton(
                        onPressed: _isLoading ? null : _submit,
                        child: _isLoading
                            ? Row(
                                mainAxisAlignment: MainAxisAlignment.center,
                                children: const [
                                  SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2.0, color: Colors.white)),
                                  SizedBox(width: 12),
                                  Text('Sending...'),
                                ],
                              )
                            : const Text('Send reset link'),
                      ),
                    ),
                    const SizedBox(height: 12),
                    TextButton(
                      onPressed: _isLoading ? null : () => Navigator.of(context).pop(),
                      child: const Text('Back to sign in'),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
