"""
Test Voice Generation: Gemini AI → ElevenLabs
Quick test to verify the complete text-to-speech pipeline
"""

import os
from dotenv import load_dotenv
from ai_coordinator import GeminiAIAnalyzer, ElevenLabsVoice

# Load environment variables
load_dotenv()

def test_gemini_only():
    """Test 1: Gemini AI text generation"""
    print("=" * 60)
    print("TEST 1: Gemini AI Text Generation")
    print("=" * 60)
    
    if not os.getenv('GEMINI_API_KEY'):
        print("❌ GEMINI_API_KEY not found in .env")
        return False
    
    try:
        analyzer = GeminiAIAnalyzer()
        
        # Simple test prompt
        test_scenario = {
            'size': 8,
            'robots': {'robot_1': [0, 0], 'robot_2': [7, 0]},
            'humans': {'human_1': [5, 4], 'human_2': [6, 6]},
            'obstacles': [[2, 2], [3, 3]],
            'exits': [[0, 7], [7, 7]],
            'blocked_paths': []
        }
        
        assignments = {
            'robot_1': {
                'target_human': 'human_1',
                'path': [[0,0], [1,1], [2,2], [3,3], [4,4], [5,4]],
                'distance': 6
            }
        }
        
        status = {
            'total_robots': 2,
            'total_humans': 2,
            'assigned_humans': 1
        }
        
        guidance = analyzer.analyze_evacuation_scenario(
            test_scenario,
            assignments,
            status,
            ['human_1']
        )
        
        print("✅ Gemini AI Response:")
        print("-" * 60)
        print(guidance)
        print("-" * 60)
        return True
        
    except Exception as e:
        print(f"❌ Gemini AI Error: {e}")
        return False


def test_elevenlabs_only():
    """Test 2: ElevenLabs text-to-speech"""
    print("\n" + "=" * 60)
    print("TEST 2: ElevenLabs Text-to-Speech")
    print("=" * 60)
    
    if not os.getenv('ELEVENLABS_API_KEY'):
        print("⚠️  ELEVENLABS_API_KEY not found in .env")
        print("   Voice generation will be skipped")
        print("   Get API key from: https://elevenlabs.io")
        return False
    
    try:
        voice = ElevenLabsVoice()
        
        test_text = """
        This is a test of the evacuation guidance system. 
        Robot 1 is en route to rescue Human 1 at position 5, 4. 
        Please remain calm and await further instructions.
        """
        
        output_file = "test_voice_output.mp3"
        success = voice.text_to_speech(test_text, output_file)
        
        if success:
            print(f"✅ Voice generated successfully!")
            print(f"   Output file: {output_file}")
            print(f"   File size: {os.path.getsize(output_file)} bytes")
            print("\n   Play with:")
            print(f"   macOS: afplay {output_file}")
            print(f"   Linux: mpg123 {output_file}")
            return True
        else:
            print("❌ Voice generation failed")
            return False
            
    except Exception as e:
        print(f"❌ ElevenLabs Error: {e}")
        return False


def test_complete_pipeline():
    """Test 3: Complete Gemini → ElevenLabs pipeline"""
    print("\n" + "=" * 60)
    print("TEST 3: Complete Pipeline (Gemini → ElevenLabs)")
    print("=" * 60)
    
    if not os.getenv('GEMINI_API_KEY'):
        print("❌ GEMINI_API_KEY not found")
        return False
    
    if not os.getenv('ELEVENLABS_API_KEY'):
        print("⚠️  ELEVENLABS_API_KEY not found - skipping voice")
        print("   Will test text generation only")
    
    try:
        # Step 1: Generate text with Gemini
        print("\nStep 1: Generating guidance text with Gemini AI...")
        analyzer = GeminiAIAnalyzer()
        
        test_scenario = {
            'size': 8,
            'robots': {'robot_1': [0, 0], 'robot_2': [7, 0]},
            'humans': {'human_1': [5, 4], 'human_2': [6, 6], 'human_3': [3, 3]},
            'obstacles': [[2, 2], [4, 4]],
            'exits': [[0, 7], [7, 7]],
            'blocked_paths': []
        }
        
        assignments = {
            'robot_1': {
                'target_human': 'human_1',
                'path': [[0,0], [1,1], [2,2], [3,3], [4,4], [5,4]],
                'distance': 6
            },
            'robot_2': {
                'target_human': 'human_2',
                'path': [[7,0], [6,1], [6,2], [6,3], [6,4], [6,5], [6,6]],
                'distance': 7
            }
        }
        
        status = {
            'total_robots': 2,
            'total_humans': 3,
            'assigned_humans': 2
        }
        
        guidance_text = analyzer.analyze_evacuation_scenario(
            test_scenario,
            assignments,
            status,
            ['human_3', 'human_1', 'human_2']
        )
        
        print("✅ Guidance text generated:")
        print("-" * 60)
        print(guidance_text)
        print("-" * 60)
        print(f"   Length: {len(guidance_text)} characters")
        
        # Step 2: Convert to speech with ElevenLabs
        if os.getenv('ELEVENLABS_API_KEY'):
            print("\nStep 2: Converting to speech with ElevenLabs...")
            voice = ElevenLabsVoice()
            
            output_file = "complete_pipeline_test.mp3"
            success = voice.text_to_speech(guidance_text, output_file)
            
            if success:
                print(f"✅ Complete pipeline successful!")
                print(f"   Text: {len(guidance_text)} characters")
                print(f"   Audio: {output_file} ({os.path.getsize(output_file)} bytes)")
                print(f"\n   Play with: afplay {output_file}")
                return True
            else:
                print("❌ Voice conversion failed")
                return False
        else:
            print("\n⚠️  Skipping voice generation (no API key)")
            print("   But text generation works! ✅")
            return True
            
    except Exception as e:
        print(f"❌ Pipeline Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    print("\n" + "╔" + "═" * 58 + "╗")
    print("║" + " " * 58 + "║")
    print("║" + "  Voice Guidance System Test".center(58) + "║")
    print("║" + "  Gemini AI → ElevenLabs".center(58) + "║")
    print("║" + " " * 58 + "║")
    print("╚" + "═" * 58 + "╝\n")
    
    # Check environment
    print("Checking environment...")
    print(f"  GEMINI_API_KEY: {'✅ Set' if os.getenv('GEMINI_API_KEY') else '❌ Not set'}")
    print(f"  ELEVENLABS_API_KEY: {'✅ Set' if os.getenv('ELEVENLABS_API_KEY') else '⚠️  Not set'}")
    print()
    
    # Run tests
    results = []
    
    results.append(("Gemini AI", test_gemini_only()))
    results.append(("ElevenLabs", test_elevenlabs_only()))
    results.append(("Complete Pipeline", test_complete_pipeline()))
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {test_name:.<40} {status}")
    
    print("=" * 60)
    
    # Recommendations
    print("\n📝 RECOMMENDATIONS:")
    if not os.getenv('GEMINI_API_KEY'):
        print("  ❌ Add GEMINI_API_KEY to .env file")
        print("     Get from: https://makersuite.google.com/app/apikey")
    
    if not os.getenv('ELEVENLABS_API_KEY'):
        print("  ⚠️  Add ELEVENLABS_API_KEY to .env file (optional)")
        print("     Get from: https://elevenlabs.io")
        print("     Free tier: 10,000 characters/month")
    
    if all(r[1] for r in results):
        print("  ✅ All systems operational!")
        print("  🎉 Voice guidance is ready for Knight Hacks 2025!")
    
    print()


if __name__ == "__main__":
    main()

