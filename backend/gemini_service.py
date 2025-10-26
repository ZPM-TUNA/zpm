#!/usr/bin/env python3
"""
Gemini AI Service - Emergency Evacuation Message Generation
"""

import os
from dotenv import load_dotenv
import google.generativeai as genai

load_dotenv()

# Initialize Gemini
GEMINI_API_KEY = os.getenv('GEMINI_API_KEY')

if GEMINI_API_KEY:
    genai.configure(api_key=GEMINI_API_KEY)
    gemini_model = genai.GenerativeModel('gemini-2.0-flash-exp')
    print("✓ Gemini AI initialized")
else:
    gemini_model = None
    print("✗ No Gemini API key found in .env")


def generate_guidance(start_position, end_position, path, grid=None):
    """
    Generate emergency evacuation guidance using Gemini AI
    
    Args:
        start_position: [x, y] - Starting position
        end_position: [x, y] - Exit position
        path: [[x1,y1], [x2,y2], ...] - Full path coordinates
        grid: 8x8 list of lists where 1=blocked, 0=open (optional)
    
    Returns:
        dict with 'success', 'guidance_text', etc.
    """
    if not gemini_model:
        return {
            'success': False,
            'error': 'Gemini API not configured'
        }
    
    try:
        # Count obstacles from grid if provided
        num_obstacles = 0
        if grid:
            for row in grid:
                num_obstacles += sum(1 for cell in row if cell == 1)
        
        # Build prompt
        prompt = f"""EMERGENCY EVACUATION - Generate clear, urgent guidance.

Current Situation:
- Person at position: {start_position}
- Exit at position: {end_position}
- Path length: {len(path)} steps
- {num_obstacles} obstacles blocking paths
- Building: 8x8 grid

Evacuation Path:
{' → '.join([str(p) for p in path[:5]])}{'...' if len(path) > 5 else ''}

Give a SHORT emergency message (2-3 sentences):
1. Clear directions (LEFT/RIGHT/UP/DOWN)
2. Urgency but calm
3. Exit location

Keep it SIMPLE and DIRECT for voice broadcast.
"""
        
        response = gemini_model.generate_content(prompt)
        guidance_text = response.text
        
        return {
            'success': True,
            'guidance_text': guidance_text,
            'path_length': len(path),
            'start': start_position,
            'end': end_position
        }
        
    except Exception as e:
        return {
            'success': False,
            'error': str(e)
        }


# ============ TEST ============
if __name__ == '__main__':
    print("\n" + "="*60)
    print(" TESTING GEMINI AI SERVICE")
    print("="*60)
    
    # Test data - 8x8 grid where 1=blocked, 0=open
    test_grid = [
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]
    ]
    
    test_data = {
        'start_position': [2, 3],
        'end_position': [7, 7],
        'path': [[2,3], [3,3], [4,4], [5,5], [6,6], [7,7]],
        'grid': test_grid
    }
    
    print("\nTest Input:")
    print(f"  Start: {test_data['start_position']}")
    print(f"  Exit: {test_data['end_position']}")
    print(f"  Path: {len(test_data['path'])} steps")
    print(f"  Grid: 8x8 (1=blocked, 0=open)")
    
    print("\nCalling Gemini AI...")
    result = generate_guidance(**test_data)
    
    print("\nResult:")
    if result['success']:
        print("✓ SUCCESS")
        print(f"\nGenerated Guidance:")
        print(f"  {result['guidance_text']}\n")
    else:
        print(f"✗ FAILED: {result['error']}")
    
    print("="*60)

