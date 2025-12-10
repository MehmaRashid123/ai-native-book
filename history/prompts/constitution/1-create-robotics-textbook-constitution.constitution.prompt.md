---
id: 1
title: Create Robotics Textbook Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "robotics", "textbook", "spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests: []
---

# Create Robotics Textbook Constitution

## User Input

```
# Constitution: Physical AI & Humanoid Robotics Textbook

## 1. Project Mission
To create a "Unified Book Project" for Panaversity that teaches "Physical AI & Humanoid Robotics." The system must bridge the gap between digital AI (LLMs/Agents) and physical embodiment (Robots). The output must be a fully functional, highly styled Docusaurus website with an embedded RAG chatbot, custom UI, and interactive learning elements.

## 2. Core Constraints & Tech Stack
- **Frontend Framework:** Docusaurus 3.x (React/TypeScript).
- **Styling Engine:** Tailwind CSS (Must replace default Docusaurus classic styles).
- **Backend API:** FastAPI (Python) for the RAG Agent and Auth handling.
- **Database:** Neon (Serverless Postgres) for user data & Qdrant (Vector DB) for content embeddings.
- **Authentication:** Better-Auth (Must support Sign-in/Sign-up for personalization).
- **Deployment:** GitHub Pages (Frontend) / Vercel (Optional for API).

## 3. UI/UX Directives (Radical Transformation)
- **Visual Identity:** "Cyberpunk/Industrial Robotics."
    - **Base:** Dark Mode by default.
    - **Accents:** Neon Green (NVIDIA), Electric Blue (ROS 2), Warning Orange (Hardware Safety).
- **Landing Page:** specialized `src/pages/index.tsx`.
    - **Restriction:** Do NOT use the default Docusaurus banner.
    - **Requirement:** Feature high-tech imagery, 3D elements (if possible), or strong typographic hierarchy emphasizing "Embodied Intelligence."
- **Navigation:** streamlined sidebar for the 4 Course Modules.

## 4. Content Architecture (Strict Adherence)
The book must follow the specific Hackathon syllabus:
- **Module 1:** The Robotic Nervous System (ROS 2, Nodes, rclpy, URDF).
- **Module 2:** The Digital Twin (Gazebo Physics, Unity Rendering, Sensors).
- **Module 3:** The AI-Robot Brain (NVIDIA Isaac Sim, VSLAM, Nav2).
- **Module 4:** Vision-Language-Action (OpenAI Whisper, LLM Cognitive Planning).
- **Capstone:** The Autonomous Humanoid.

**Code Guidelines:**
- All Robot control code must be Python-based (`rclpy`).
- Use Mermaid.js for all architecture diagrams (ROS Nodes/Topics).

## 5. Intelligence & Agent Guidelines (The "Vector" Agent)
- **Role:** The embedded chatbot acts as a "Lab Assistant."
- **Capabilities:**
    1.  **RAG:** Answer questions based strictly on the book content.
    2.  **Context-Awareness:** Know which chapter the user is reading.
    3.  **Hardware Checks:** If a user mentions "MacBook" or "Raspberry Pi," the Agent must warn them about NVIDIA Isaac Sim requirements (RTX GPU required).
- **Personalization:** If a user logs in, the Agent must adapt answers based on their defined hardware background (as stored in Better-Auth).

## 6. Language & Localization
- **Primary Language:** English (Technical).
- **Translation:** The system must support an "Urdu Translation" toggle at the start of every chapter (Bonus Point Requirement).

## 7. Workflow Protocol
1.  **UI First:** Establish the visual theme before adding content.
2.  **Step-by-Step:** Build Module 1, then Module 2, etc.
3.  **Verification:** Every code block must be syntactically correct for ROS 2 Humble/Iron.
```

## Response

Updated the project constitution to reflect the Physical AI & Humanoid Robotics Textbook requirements, creating a comprehensive constitution with 6 core principles:

1. Physical AI & Humanoid Robotics Education
2. Docusaurus Frontend Architecture
3. Cyberpunk Industrial UI/UX
4. ROS 2 Python Code Standards
5. RAG Agent Intelligence
6. Localization & Translation Requirements

Also added sections for Content Architecture, Intelligence & Agent Guidelines, and Language & Localization. Updated the template files (plan, spec, tasks) to align with the new constitution requirements, including specific compliance checks and tasks related to the robotics textbook project.

## Outcome

The constitution has been successfully created with version 1.1.0, and all related templates have been updated to ensure consistency across the project. The constitution captures all the specified requirements including the tech stack, UI/UX directives, content architecture, agent guidelines, and localization requirements.

## Evaluation

The implementation successfully transforms the generic template into a specific constitution for the Physical AI & Humanoid Robotics Textbook project. All requirements from the user input have been incorporated, and the templates have been updated to enforce compliance with the new principles.