import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';

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
                // Use the app seed color (matches Sign in button)
                colorSchemeSeed: const Color(0xFFFF7043),
                useMaterial3: true,
                // ensure icons use the brand color rather than a brownish fallback
                iconTheme: const IconThemeData(color: Color(0xFFFF7043)),
                appBarTheme: const AppBarTheme(iconTheme: IconThemeData(color: Color(0xFFFF7043))),
                listTileTheme: const ListTileThemeData(iconColor: Color(0xFFFF7043)),
                bottomNavigationBarTheme: const BottomNavigationBarThemeData(selectedItemColor: Color(0xFFFF7043)),
                floatingActionButtonTheme: const FloatingActionButtonThemeData(backgroundColor: Color(0xFFFF7043)),
                textTheme: GoogleFonts.interTextTheme(),
                primaryTextTheme: GoogleFonts.interTextTheme(),
              ),
              darkTheme: ThemeData(
                brightness: Brightness.dark,
                colorSchemeSeed: const Color(0xFFFF7043),
                useMaterial3: true,
                // dark theme icons: also use the brand color for emphasis
                iconTheme: const IconThemeData(color: Color(0xFFFF7043)),
                appBarTheme: const AppBarTheme(iconTheme: IconThemeData(color: Color(0xFFFF7043))),
                listTileTheme: const ListTileThemeData(iconColor: Color(0xFFFF7043)),
                bottomNavigationBarTheme: const BottomNavigationBarThemeData(selectedItemColor: Color(0xFFFF7043)),
                floatingActionButtonTheme: const FloatingActionButtonThemeData(backgroundColor: Color(0xFFFF7043)),
                textTheme: GoogleFonts.interTextTheme(ThemeData(brightness: Brightness.dark).textTheme),
                primaryTextTheme: GoogleFonts.interTextTheme(ThemeData(brightness: Brightness.dark).primaryTextTheme),
              ),
            themeMode: themeProvider.isDarkMode ? ThemeMode.dark : ThemeMode.light,
            // Ensure icons pick up the computed colorScheme.primary after the theme is applied
            builder: (context, child) {
              final scheme = Theme.of(context).colorScheme;
              return IconTheme(data: IconThemeData(color: scheme.primary), child: child ?? const SizedBox.shrink());
            },
            home: const LoginScreen(),
          );
        },
      ),
    );
  }
}

