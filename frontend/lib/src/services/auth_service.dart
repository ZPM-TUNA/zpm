import 'dart:convert';
import 'package:google_sign_in/google_sign_in.dart';
import 'package:shared_preferences/shared_preferences.dart';

class User {
  final String id;
  final String email;
  final String name;
  final String? photoUrl;

  User({
    required this.id,
    required this.email,
    required this.name,
    this.photoUrl,
  });

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'email': email,
      'name': name,
      'photoUrl': photoUrl,
    };
  }

  factory User.fromJson(Map<String, dynamic> json) {
    return User(
      id: json['id'],
      email: json['email'],
      name: json['name'],
      photoUrl: json['photoUrl'],
    );
  }
}

class AuthService {
  static final AuthService _instance = AuthService._internal();
  factory AuthService() => _instance;
  AuthService._internal();

  final GoogleSignIn _googleSignIn = GoogleSignIn(
    scopes: ['email', 'profile'],
    // For development, we'll use a mock approach
    // In production, uncomment the real Google Sign-In code below
  );

  User? _currentUser;
  bool _isInitialized = false;

  User? get currentUser => _currentUser;
  bool get isSignedIn => _currentUser != null;

  Future<void> initialize() async {
    if (_isInitialized) return;

    try {
      // For development, just load from storage
      // In production, uncomment the real Google Sign-In code below
      await _loadUserFromStorage();

      // Real Google Sign-In code (uncomment for production):
      /*
      // Check if user is already signed in
      final GoogleSignInAccount? googleUser = await _googleSignIn.signInSilently();
      if (googleUser != null) {
        _currentUser = User(
          id: googleUser.id,
          email: googleUser.email,
          name: googleUser.displayName ?? '',
          photoUrl: googleUser.photoUrl,
        );
        await _saveUserToStorage(_currentUser!);
      } else {
        // Try to load from storage
        await _loadUserFromStorage();
      }
      */
    } catch (e) {
      print('Error initializing auth: $e');
    }

    _isInitialized = true;
  }

  Future<User?> signInWithGoogle() async {
    try {
      // For development, simulate Google Sign-In
      // In production, uncomment the real Google Sign-In code below
      await Future.delayed(const Duration(milliseconds: 1000)); // Simulate network delay

      // Mock user data for development
      _currentUser = User(
        id: 'dev_user_123',
        email: 'demo@zeropanic.com',
        name: 'Demo User',
        photoUrl: 'https://via.placeholder.com/150/FF7043/FFFFFF?text=DU',
      );

      await _saveUserToStorage(_currentUser!);
      return _currentUser;

      // Real Google Sign-In code (uncomment for production):
      /*
      final GoogleSignInAccount? googleUser = await _googleSignIn.signIn();
      if (googleUser == null) {
        return null; // User cancelled sign in
      }

      _currentUser = User(
        id: googleUser.id,
        email: googleUser.email,
        name: googleUser.displayName ?? '',
        photoUrl: googleUser.photoUrl,
      );

      await _saveUserToStorage(_currentUser!);
      return _currentUser;
      */
    } catch (e) {
      print('Error signing in with Google: $e');
      return null;
    }
  }

  Future<void> signOut() async {
    try {
      // For development, just clear local data
      // In production, uncomment the real Google Sign-In code below
      _currentUser = null;
      await _clearUserFromStorage();

      // Real Google Sign-In code (uncomment for production):
      // await _googleSignIn.signOut();
    } catch (e) {
      print('Error signing out: $e');
    }
  }

  Future<void> deleteAccount() async {
    try {
      // For development, just clear local data
      // In production, uncomment the real Google Sign-In code below
      _currentUser = null;
      await _clearUserFromStorage();

      // Real Google Sign-In code (uncomment for production):
      // await _googleSignIn.signOut();
    } catch (e) {
      print('Error deleting account: $e');
    }
  }

  Future<void> _saveUserToStorage(User user) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('user_data', json.encode(user.toJson()));
    } catch (e) {
      print('Error saving user to storage: $e');
    }
  }

  Future<void> _loadUserFromStorage() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final userData = prefs.getString('user_data');
      if (userData != null) {
        final userJson = json.decode(userData) as Map<String, dynamic>;
        _currentUser = User.fromJson(userJson);
      }
    } catch (e) {
      print('Error loading user from storage: $e');
    }
  }

  Future<void> _clearUserFromStorage() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.remove('user_data');
    } catch (e) {
      print('Error clearing user from storage: $e');
    }
  }
}