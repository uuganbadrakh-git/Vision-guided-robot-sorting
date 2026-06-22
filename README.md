# Vision guided robot sorting

# Vision guided robot sorting

This repository contains the project files for a vision-guided robot sorting system. It is structured for developers who want a clean starting point for perception, planning, and robotics integration.

## Overview

The repository is built around a mobile robot sorting workflow:

- Vision: detect and classify objects from camera input.
- Planning: compute sorting targets and pick-and-place assignments.
- Execution: connect the perception pipeline to robotic motion control.

## Quick start

1. Clone the repository:

   git clone https://github.com/uuganbadrakh-git/Vision-guided-robot-sorting.git
   cd Vision-guided-robot-sorting

2. Create and activate a virtual environment:

   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt

3. Run the demo scaffold:

   python3 src/demo.py --config config/demo.yaml

## Project structure

- `src/` — basic code scaffold and demo entrypoint
- `config/` — example configuration files
- `requirements.txt` — Python dependencies for vision and robotics workflows
- `vision-guided-robot-sorting.pdf` — imported project document
- `LICENSE` — MIT license

## Usage

The demo runner is a placeholder for the actual system entrypoint. Replace `src/demo.py` with your main application script and extend the `src/` package as needed.

## Extend this project

- Add camera and sensor drivers under `src/`.
- Put vision model definitions in `src/vision`.
- Add planner and robot controllers in `src/robot`.
- Use `models/` and `data/` folders for model weights and test datasets.

## License

This project is released under the MIT License. See the `LICENSE` file for details.
