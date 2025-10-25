#!/bin/bash

echo "Testing health endpoint..."
curl http://localhost:5000/health
echo -e "\n\n"

echo "Testing robot detection with image path..."
curl -X POST http://localhost:5000/detect-robots \
  -H "Content-Type: application/json" \
  -d '{"image_path": "robot_photos_jpg/robot_0.jpg"}'
echo -e "\n"
