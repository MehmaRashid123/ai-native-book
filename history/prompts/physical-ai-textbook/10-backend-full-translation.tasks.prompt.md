---
id: 10
title: Backend Full Translation Update
stage: tasks
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-textbook
branch: 01-physical-ai-textbook
user: user
command: /sp.tasks
labels: ["backend", "translation", "urdu", "skills"]
links:
  spec: ../specs/01-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/skills/translate.py
  - specs/01-physical-ai-textbook/tasks-backend-full-translation.md
tests: []
---

# Backend Full Translation Update

## User Input

```
"Update Backend with Full Content Translation"
Please update `backend/skills/translate.py` (or create it) to include full paragraph translations for the demo.

1. UPDATE `backend/skills/translate.py`:
   - Define a function `to_urdu(text: str) -> str`.
   - Inside, create a `TRANSLATION_MEMORY` dictionary.
   - Add these EXACT mappings (English -> Urdu):

     "Welcome to the foundation of robotics - the Robot Operating System (ROS 2). In this module, you'll learn how robots communicate and coordinate their actions through a distributed network of nodes, topics, and services.": "Robotics ki bunyad mein khushamdeed - Robot Operating System (ROS 2). Is module mein aap seekhenge ke robots kis tarah nodes, topics aur services ke distributed network ke zariye aapas mein raabta aur amal karte hain.",

     "The fundamentals of ROS 2 architecture": "ROS 2 architecture ke bunyadi usool",

     "How to create and manage ROS 2 nodes": "ROS 2 nodes ko banana aur manage karna",

     "The publisher-subscriber communication pattern": "Publisher-subscriber raabtay ka tareeqa",

     "Using rclpy for Python-based robot control": "Python-based robot control ke liye rclpy ka istemal",

     "Robot Description Format (URDF) for robot modeling": "Robot modeling ke liye Robot Description Format (URDF)",

     "Basic Python programming knowledge": "Bunyadi Python programming ka ilm",

     "Understanding of object-oriented programming concepts": "Object-oriented programming concepts ki samajh",

     "What You'll Learn": "Aap Kya Seekhenge",

     "Prerequisites": "Zarooriat",

     "Getting Started": "Shuru Karein"

   - **LOGIC:**
     - Clean the input text (strip whitespace).
     - Check if text exists in `TRANSLATION_MEMORY`.
     - If yes, return the Urdu version.
     - If no, return a generic fallback: `f"[AI] {text} (Urdu)"` (so we know it tried).

2. VERIFICATION:
   - Ensure `backend/main.py` is importing and using this `to_urdu` function correctly.

Go ahead and update the backend logic.
```

## Response

Successfully updated the backend translation skill with comprehensive translation memory:

1. Updated `backend/skills/translate.py` with TRANSLATION_MEMORY dictionary containing exact requested mappings
2. Implemented text cleaning logic (strip whitespace)
3. Added lookup logic to check translation memory first
4. Added fallback mechanism for unrecognized text
5. Verified backend/main.py correctly imports and uses the to_urdu function

## Outcome

The backend translation skill now includes comprehensive mappings for educational content with proper fallback mechanisms, ready for integration with the frontend translation component.

## Evaluation

The implementation successfully creates a robust translation system with specific mappings for technical content and graceful fallback for unrecognized text.