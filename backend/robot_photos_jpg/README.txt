Robot Toy Detection Images

This folder contains sample images for Roboflow detection:
- robot_0.jpg through robot_74.jpg: Toy robot images
- fake.jpg: Test image

These images are used by mock_detection_server.py to test
the Roboflow workflow: "find-toys-robots-and-figurines"

The detection server randomly selects images from this folder
to simulate camera captures and runs them through Roboflow's
toy/robot/figurine detection workflow.

API Configuration:
- Workspace: robotdetector
- Workflow: find-toys-robots-and-figurines
- API Key: Set in backend/.env as ROBOFLOW_API_KEY

