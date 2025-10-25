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
              // Temporarily do not seed the global color scheme. Keep explicit brand colors for icons/buttons.
              theme: ThemeData(
                brightness: Brightness.light,
                useMaterial3: true,
                // explicit brand usages
                iconTheme: const IconThemeData(color: Color(0xFFFF7043)),
                appBarTheme: const AppBarTheme(iconTheme: IconThemeData(color: Color(0xFFFF7043))),
                listTileTheme: const ListTileThemeData(iconColor: Color(0xFFFF7043)),
                bottomNavigationBarTheme: const BottomNavigationBarThemeData(selectedItemColor: Color(0xFFFF7043)),
                floatingActionButtonTheme: const FloatingActionButtonThemeData(backgroundColor: Color(0xFFFF7043)),
                elevatedButtonTheme: ElevatedButtonThemeData(
                  style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFFFF7043), foregroundColor: Colors.white),
                ),
                textTheme: GoogleFonts.interTextTheme(),
                primaryTextTheme: GoogleFonts.interTextTheme(),
              ),
              darkTheme: ThemeData(
                brightness: Brightness.dark,
                useMaterial3: true,
                // explicit brand usages for dark mode as well
                iconTheme: const IconThemeData(color: Color(0xFFFF7043)),
                appBarTheme: const AppBarTheme(iconTheme: IconThemeData(color: Color(0xFFFF7043))),
                listTileTheme: const ListTileThemeData(iconColor: Color(0xFFFF7043)),
                bottomNavigationBarTheme: const BottomNavigationBarThemeData(selectedItemColor: Color(0xFFFF7043)),
                floatingActionButtonTheme: const FloatingActionButtonThemeData(backgroundColor: Color(0xFFFF7043)),
                elevatedButtonTheme: ElevatedButtonThemeData(
                  style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFFFF7043), foregroundColor: Colors.white),
                ),
                textTheme: GoogleFonts.interTextTheme(ThemeData(brightness: Brightness.dark).textTheme),
                primaryTextTheme: GoogleFonts.interTextTheme(ThemeData(brightness: Brightness.dark).primaryTextTheme),
              ),
            themeMode: themeProvider.isDarkMode ? ThemeMode.dark : ThemeMode.light,
            // Do not override IconTheme here so explicit iconTheme values apply.
            home: const LoginScreen(),
          );
        },
      ),
    );
  }
}

