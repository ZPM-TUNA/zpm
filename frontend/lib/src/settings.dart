import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'models/theme_provider.dart';
import 'login.dart';

class SettingsScreen extends StatelessWidget {
  const SettingsScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings'),
        // Show a back arrow when this route can be popped
        leading: Navigator.canPop(context) ? const BackButton() : null,
      ),
      body: SafeArea(child: _SettingsScreenBody()),
    );
  }
}

class _SettingsScreenBody extends StatefulWidget {
  const _SettingsScreenBody({super.key});

  @override
  State<_SettingsScreenBody> createState() => _SettingsScreenBodyState();
}

class _SettingsScreenBodyState extends State<_SettingsScreenBody> {
  final _formKey = GlobalKey<FormState>();
  final TextEditingController _current = TextEditingController();
  final TextEditingController _new = TextEditingController();
  final TextEditingController _confirm = TextEditingController();
  bool _loading = false;
  bool _deleting = false;

  @override
  void dispose() {
    _current.dispose();
    _new.dispose();
    _confirm.dispose();
    super.dispose();
  }

  Future<void> _changePassword() async {
    if (!(_formKey.currentState?.validate() ?? false)) return;
    setState(() => _loading = true);

    // TODO: Replace this mock verification with a secure backend call.
    await Future.delayed(const Duration(milliseconds: 700));
    final currentVal = _current.text;
    if (currentVal != 'password123') {
      if (!mounted) return;
      setState(() => _loading = false);
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Current password is incorrect')));
      return;
    }

    // Simulate successful change
    setState(() => _loading = false);
    _current.clear();
    _new.clear();
    _confirm.clear();
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Password changed successfully')));
  }

  Future<void> _confirmDelete() async {
    final should = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Delete account'),
        content: const Text('This will permanently delete your account and cannot be undone. Are you sure you want to continue?'),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(false), child: const Text('Cancel')),
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(true),
            style: TextButton.styleFrom(foregroundColor: Colors.red),
            child: const Text('Delete'),
          ),
        ],
      ),
    );

    if (should == true) {
      await _deleteAccount();
    }
  }

  Future<void> _deleteAccount() async {
    setState(() => _deleting = true);
    // TODO: call backend to delete user account; this is a placeholder.
    await Future.delayed(const Duration(milliseconds: 800));
    setState(() => _deleting = false);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Account deleted (demo)')));

    // Navigate back to login and clear navigation stack
    Navigator.of(context).pushAndRemoveUntil(
      MaterialPageRoute(builder: (_) => const LoginScreen()),
      (route) => false,
    );
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.select<ThemeProvider, bool>((p) => p.isDarkMode);

    final theme = Theme.of(context);

    return SingleChildScrollView(
      padding: const EdgeInsets.all(12),
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 720),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // compact spacing to match other screens
            const SizedBox(height: 12),

            Card(
              child: Padding(
                padding: const EdgeInsets.all(12.0),
                child: Form(
                  key: _formKey,
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      const Text('Change password', style: TextStyle(fontWeight: FontWeight.w600)),
                      const SizedBox(height: 8),
                      TextFormField(
                        controller: _current,
                        obscureText: true,
                        decoration: const InputDecoration(labelText: 'Current password'),
                        validator: (v) => (v == null || v.isEmpty) ? 'Enter current password' : null,
                      ),
                      const SizedBox(height: 8),
                      TextFormField(
                        controller: _new,
                        obscureText: true,
                        decoration: const InputDecoration(labelText: 'New password'),
                        validator: (v) {
                          if (v == null || v.isEmpty) return 'Enter a new password';
                          if (v.length < 8) return 'Password must be at least 8 characters';
                          return null;
                        },
                      ),
                      const SizedBox(height: 8),
                      TextFormField(
                        controller: _confirm,
                        obscureText: true,
                        decoration: const InputDecoration(labelText: 'Confirm new password'),
                        validator: (v) {
                          if (v == null || v.isEmpty) return 'Confirm the new password';
                          if (v != _new.text) return 'Passwords do not match';
                          return null;
                        },
                      ),
                      const SizedBox(height: 12),
                      ElevatedButton(
                        onPressed: _loading ? null : _changePassword,
                        child: _loading ? const SizedBox(height: 18, width: 18, child: CircularProgressIndicator(strokeWidth: 2)) : const Text('Change password'),
                      ),
                    ],
                  ),
                ),
              ),
            ),

            const SizedBox(height: 12),

            Card(
              child: Column(
                children: [
                  SwitchListTile(
                    title: const Text('Dark mode'),
                    subtitle: const Text('Toggle between light and dark themes'),
                    value: isDark,
                    onChanged: (v) {
                      context.read<ThemeProvider>().toggleDarkMode(v);
                    },
                  ),
                ],
              ),
            ),
            const SizedBox(height: 16),

            // Delete account button
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 4.0),
              child: ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor: theme.colorScheme.error, // red
                  foregroundColor: theme.colorScheme.onError,
                ),
                onPressed: _deleting ? null : _confirmDelete,
                child: _deleting
                    ? const SizedBox(height: 18, width: 18, child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white))
                    : const Padding(
                        padding: EdgeInsets.symmetric(vertical: 12),
                        child: Text('Delete account', style: TextStyle(fontWeight: FontWeight.bold)),
                      ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}