import 'package:flutter/foundation.dart';

class RobotData {
  final String id;
  final List<int> position;
  final String status; // exploring, mapping, idle
  final int battery;
  final List<List<int>> exploredArea;

  RobotData({
    required this.id,
    required this.position,
    required this.status,
    required this.battery,
    required this.exploredArea,
  });

  factory RobotData.fromJson(Map<String, dynamic> json) {
    return RobotData(
      id: json['id'] ?? '',
      position: List<int>.from(json['position'] ?? [0, 0]),
      status: json['status'] ?? 'unknown',
      battery: json['battery'] ?? 0,
      exploredArea: (json['explored_area'] as List?)
              ?.map((p) => List<int>.from(p))
              .toList() ??
          [],
    );
  }
}

class HumanData {
  final String id;
  final List<int> position;
  final List<List<int>> evacuationPath;
  final String priority;
  final int distanceToExit;
  final List<int> exitTarget;

  HumanData({
    required this.id,
    required this.position,
    required this.evacuationPath,
    required this.priority,
    required this.distanceToExit,
    required this.exitTarget,
  });

  factory HumanData.fromJson(Map<String, dynamic> json) {
    return HumanData(
      id: json['id'] ?? '',
      position: List<int>.from(json['position'] ?? [0, 0]),
      evacuationPath: (json['evacuation_path'] as List?)
              ?.map((p) => List<int>.from(p))
              .toList() ??
          [],
      priority: json['priority'] ?? 'low',
      distanceToExit: json['distance_to_exit'] ?? 0,
      exitTarget: List<int>.from(json['exit_target'] ?? [0, 0]),
    );
  }
}

class GuidanceData {
  final String text;
  final String? voiceFile;
  final String timestamp;

  GuidanceData({
    required this.text,
    this.voiceFile,
    required this.timestamp,
  });

  factory GuidanceData.fromJson(Map<String, dynamic> json) {
    return GuidanceData(
      text: json['text'] ?? '',
      voiceFile: json['voice_file'],
      timestamp: json['timestamp'] ?? DateTime.now().toIso8601String(),
    );
  }
}

class MazeData {
  final int size;
  final List<List<int>> obstacles;
  final List<List<int>> exits;

  MazeData({
    required this.size,
    required this.obstacles,
    required this.exits,
  });

  factory MazeData.fromJson(Map<String, dynamic> json) {
    return MazeData(
      size: json['size'] ?? 8,
      obstacles: (json['obstacles'] as List?)
              ?.map((o) => List<int>.from(o))
              .toList() ??
          [],
      exits: (json['exits'] as List?)
              ?.map((e) => List<int>.from(e))
              .toList() ??
          [],
    );
  }
}

class EvacuationState extends ChangeNotifier {
  List<RobotData> _robots = [];
  List<HumanData> _humans = [];
  GuidanceData? _guidance;
  MazeData? _maze;
  bool _isLoading = false;
  String? _error;

  List<RobotData> get robots => _robots;
  List<HumanData> get humans => _humans;
  GuidanceData? get guidance => _guidance;
  MazeData? get maze => _maze;
  bool get isLoading => _isLoading;
  String? get error => _error;

  void updateFromFlutterData(Map<String, dynamic> data) {
    try {
      _robots = (data['robots'] as List?)
              ?.map((r) => RobotData.fromJson(r))
              .toList() ??
          [];
      
      _humans = (data['humans'] as List?)
              ?.map((h) => HumanData.fromJson(h))
              .toList() ??
          [];
      
      if (data['guidance'] != null) {
        _guidance = GuidanceData.fromJson(data['guidance']);
      }
      
      if (data['maze'] != null) {
        _maze = MazeData.fromJson(data['maze']);
      }
      
      _error = null;
      notifyListeners();
    } catch (e) {
      _error = 'Failed to parse data: $e';
      notifyListeners();
    }
  }

  void setLoading(bool loading) {
    _isLoading = loading;
    notifyListeners();
  }

  void setError(String? error) {
    _error = error;
    notifyListeners();
  }

  void clearError() {
    _error = null;
    notifyListeners();
  }
}

