import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'login.dart';
import 'models/evacuation_state.dart';
import 'models/theme_provider.dart';

class ZeroPanicApp extends StatelessWidget {
  const ZeroPanicApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => EvacuationState()),
        ChangeNotifierProvider(create: (_) => ThemeProvider()),
      ],
      child: Consumer<ThemeProvider>(
        builder: (context, themeProvider, _) {
          return MaterialApp(
            title: 'ZeroPanic',
              theme: ThemeData(
                brightness: Brightness.light,
                // Use a more vivid red seed to avoid brownish tones
                colorSchemeSeed: const Color(0xFFFF3D3D),
                useMaterial3: true,
              ),
              darkTheme: ThemeData(
                brightness: Brightness.dark,
                colorSchemeSeed: const Color(0xFFFF3D3D),
                useMaterial3: true,
              ),
            themeMode: themeProvider.isDarkMode ? ThemeMode.dark : ThemeMode.light,
            home: const LoginScreen(),
          );
        },
      ),
    );
  }
}

