import os
import requests
from dotenv import load_dotenv

load_dotenv()

API_KEY = os.getenv("ELEVENLABS_API_KEY")

VOICE_ID = "21m00Tcm4TlvDq8ikWAM"  # Rachel

text = """
Attention! Hello there! A fire has been reported in the building.
Please remain calm and evacuate immediately.
Use the nearest exit and do not use the elevators.
Assist others if safe to do so. Evacuate now!
"""

url = f"https://api.elevenlabs.io/v1/text-to-speech/{VOICE_ID}"

payload = {
    "text": text,
    "model_id": "eleven_multilingual_v2",
    "voice_settings": {
        "stability": 0.15,
        "similarity_boost": 0.8,
        "style": 1.0,
        "use_speaker_boost": True
    }
}

headers = {
    "xi-api-key": API_KEY,
    "Content-Type": "application/json"
}

response = requests.post(url, json=payload, headers=headers)

if response.status_code == 200:
    with open("fire_emergency_voice.mp3", "wb") as f:
        f.write(response.content)
    print("Voice generated & saved as fire_emergency_voice.mp3")
else:
    print("Error:", response.status_code, response.text)
