import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'login.dart';
import 'models/evacuation_state.dart';

class ZeroPanicApp extends StatelessWidget {
  const ZeroPanicApp({super.key});

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (_) => EvacuationState(),
      child: MaterialApp(
        title: 'ZeroPanic',
        theme: ThemeData(primarySwatch: Colors.red, useMaterial3: true),
        home: const LoginScreen(),
      ),
    );
  }
}

