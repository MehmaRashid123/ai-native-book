---
id: 4
title: Create Physical AI Textbook Tasks
stage: tasks
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-textbook
branch: 01-physical-ai-textbook
user: user
command: /sp.tasks
labels: ["tasks", "skills", "implementation", "robotics"]
links:
  spec: ../specs/01-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .claude/skills/hardware_check.py
  - .claude/skills/ros2_codegen.py
  - .claude/skills/rag_retrieval.py
  - .claude/skills/translate.py
  - .claude/skills/__init__.py
  - specs/01-physical-ai-textbook/tasks.md
tests: []
---

# Create Physical AI Textbook Tasks

## User Input

```
Create the "Reusable Intelligence" layer for the Physical AI Textbook project.

1. Create a directory named `.claude/skills` if it doesn't exist.
2. Inside `.claude/skills`, create the following 4 Python scripts with the specific logic defined below:

FILE 1: `.claude/skills/hardware_check.py`
- Goal: Secure Bonus Points for Personalization.
- Function: `validate_gpu(gpu_model: str, ram_gb: int)`
- Logic:
  - If `gpu_model` does not contain "RTX" or "A10", return a WARNING that "NVIDIA Isaac Sim requires RTX GPU."
  - If `gpu_model` implies MacOS (e.g., "M1", "M2", "Apple"), return a CRITICAL error that "Isaac Sim does not support MacOS."
  - If `ram_gb` < 32, warn about potential crashes.
- Output: Return a JSON string with `status` (PASS/WARN/FAIL) and `message`.

FILE 2: `.claude/skills/ros2_codegen.py`
- Goal: Secure Bonus Points for Reusable Intelligence.
- Function: `generate_node(name: str, topic: str, node_type: str)`
- Logic:
  - Generate a valid Python string containing a full ROS 2 Node using `rclpy`.
  - Include imports, class definition inheriting from `Node`, `__init__`, and `main` entry point.
  - If `node_type` is "publisher", create a timer and publisher.
  - If `node_type` is "subscriber", create a subscription callback.

FILE 3: `.claude/skills/rag_retrieval.py`
- Goal: Base requirement for the Chatbot.
- Function: `search_knowledge_base(query: str)`
- Logic:
  - Create a placeholder function that simulates searching a Vector Database (Qdrant).
  - If query contains "ROS" or "Node", return a mock string about "Module 1: ROS 2 Architecture."
  - If query contains "Isaac" or "Sim", return a mock string about "Module 3: NVIDIA Isaac Sim Requirements."

FILE 4: `.claude/skills/translate.py`
- Goal: Secure Bonus Points for Urdu Translation.
- Function: `to_urdu(text: str)`
- Logic:
  - Create a placeholder function that accepts English text.
  - Return a string that mimics translation (e.g., prepend "[Urdu Translation]: ").
  - Add a comment that in production, this would call an LLM API to preserve technical keywords like "Lidar" and "SLAM".

3. Finally, create an `__init__.py` in that folder to export these functions so they can be imported as a module.
```

## Response

Created the complete "Reusable Intelligence" layer for the Physical AI Textbook project with:

1. Created the `.claude/skills` directory structure
2. Implemented 4 Python skill files:
   - `hardware_check.py` with `validate_gpu` function for hardware compatibility checks
   - `ros2_codegen.py` with `generate_node` function for ROS 2 code generation
   - `rag_retrieval.py` with `search_knowledge_base` function for RAG functionality
   - `translate.py` with `to_urdu` function for Urdu translation
3. Created `__init__.py` to export all skill functions as a module
4. Generated comprehensive tasks.md file with 32 specific tasks organized by user story priority

The skills module provides the foundation for the Vector RAG agent with hardware validation, code generation, knowledge retrieval, and localization capabilities.

## Outcome

The Reusable Intelligence layer is fully implemented with all required skills and a comprehensive task breakdown for further development. The system supports personalization (hardware checks), reusable intelligence (code generation), core chatbot functionality (RAG), and localization (Urdu translation).

## Evaluation

The implementation successfully creates the reusable intelligence layer as requested, with well-structured, testable functions that align with the project's requirements for personalization, code generation, RAG, and localization.