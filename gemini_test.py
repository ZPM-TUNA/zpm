import google.generativeai as genai
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configure the API key
genai.configure(api_key=os.getenv('GEMINI_API_KEY'))

# Create the model
model = genai.GenerativeModel('gemini-2.0-flash-exp')

# Generate content
response = model.generate_content("Explain how AI works in a few words")
print(response.text)