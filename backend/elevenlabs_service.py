#!/usr/bin/env python3
"""
ElevenLabs Service - Text to Speech Conversion
"""

import os
from dotenv import load_dotenv
import requests

load_dotenv()

# Configuration
ELEVENLABS_API_KEY = os.getenv('ELEVENLABS_API_KEY')
ELEVENLABS_VOICE_ID = os.getenv('ELEVENLABS_VOICE_ID', 'EXAVITQu4vr4xnSDxMaL')

if ELEVENLABS_API_KEY:
    print("✓ ElevenLabs API initialized")
else:
    print("✗ No ElevenLabs API key found in .env")


def text_to_speech(text, output_file='output_audio.mp3', voice_id=None):
    """
    Convert text to speech using ElevenLabs
    
    Args:
        text: str - Text to convert
        output_file: str - Output MP3 filename
        voice_id: str - Optional voice ID (uses default if not provided)
    
    Returns:
        dict with 'success', 'audio_file', etc.
    """
    if not ELEVENLABS_API_KEY:
        return {
            'success': False,
            'error': 'ElevenLabs API not configured'
        }
    
    if not text:
        return {
            'success': False,
            'error': 'No text provided'
        }
    
    try:
        # Use provided voice or default
        vid = voice_id or ELEVENLABS_VOICE_ID
        
        # Call ElevenLabs API
        url = f"https://api.elevenlabs.io/v1/text-to-speech/{vid}"
        headers = {
            'xi-api-key': ELEVENLABS_API_KEY,
            'Content-Type': 'application/json'
        }
        payload = {
            'text': text,
            'model_id': 'eleven_monolingual_v1',
            'voice_settings': {
                'stability': 0.5,
                'similarity_boost': 0.75
            }
        }
        
        response = requests.post(url, headers=headers, json=payload)
        
        if response.status_code == 200:
            # Save audio file
            with open(output_file, 'wb') as f:
                f.write(response.content)
            
            return {
                'success': True,
                'audio_file': output_file,
                'text_length': len(text)
            }
        else:
            return {
                'success': False,
                'error': response.text,
                'status_code': response.status_code
            }
            
    except Exception as e:
        return {
            'success': False,
            'error': str(e)
        }


# ============ TEST ============
if __name__ == '__main__':
    print("\n" + "="*60)
    print(" TESTING ELEVENLABS SERVICE")
    print("="*60)
    
    test_text = "Emergency evacuation. Move to the right exit immediately. Stay calm and proceed quickly."
    
    print(f"\nTest Input:")
    print(f"  Text: \"{test_text[:50]}...\"")
    print(f"  Length: {len(test_text)} characters")
    
    print("\nCalling ElevenLabs...")
    result = text_to_speech(test_text, output_file='test_elevenlabs_output.mp3')
    
    print("\nResult:")
    if result['success']:
        print("✓ SUCCESS")
        print(f"  Audio saved to: {result['audio_file']}")
        print(f"  Text length: {result['text_length']} chars")
        print(f"\n  Play with: afplay {result['audio_file']}")
    else:
        print(f"✗ FAILED: {result['error']}")
    
    print("="*60)

