import 'package:flutter/foundation.dart';

class ThemeProvider extends ChangeNotifier {
  bool _isDarkMode = false;

  bool get isDarkMode => _isDarkMode;

  void toggleDarkMode([bool? value]) {
    _isDarkMode = value ?? !_isDarkMode;
    notifyListeners();
  }
}
