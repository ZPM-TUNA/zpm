#!/usr/bin/env python3
"""
Simple audio player for evacuation guidance
"""
import os
import sys
import subprocess
import time

def play_audio_file(filename):
    """Play audio file using system player"""
    if not os.path.exists(filename):
        print(f"⚠️  Audio file not found: {filename}")
        return False
    
    try:
        # macOS
        if sys.platform == 'darwin':
            subprocess.run(['afplay', filename], check=True)
        # Linux
        elif sys.platform.startswith('linux'):
            subprocess.run(['aplay', filename], check=True)
        # Windows
        elif sys.platform == 'win32':
            import winsound
            winsound.PlaySound(filename, winsound.SND_FILENAME)
        
        print(f"🔊 Played: {filename}")
        return True
    except Exception as e:
        print(f"⚠️  Could not play audio: {e}")
        return False

def monitor_and_play():
    """Monitor for new evacuation guidance files and play them"""
    print("🎧 Audio monitor started...")
    print("   Watching for evacuation_guidance.mp3")
    
    last_modified = 0
    
    while True:
        try:
            if os.path.exists('evacuation_guidance.mp3'):
                current_modified = os.path.getmtime('evacuation_guidance.mp3')
                
                if current_modified > last_modified:
                    last_modified = current_modified
                    print("\n🔊 New guidance detected!")
                    play_audio_file('evacuation_guidance.mp3')
            
            time.sleep(1)
        except KeyboardInterrupt:
            print("\n👋 Audio monitor stopped")
            break
        except Exception as e:
            print(f"⚠️  Error: {e}")
            time.sleep(1)

if __name__ == '__main__':
    monitor_and_play()
