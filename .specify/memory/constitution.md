<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: [PRINCIPLE_1_NAME] → Physical AI & Humanoid Robotics Education, [PRINCIPLE_2_NAME] → Docusaurus Frontend Architecture, [PRINCIPLE_3_NAME] → Cyberpunk Industrial UI/UX, [PRINCIPLE_4_NAME] → ROS 2 Python Code Standards, [PRINCIPLE_5_NAME] → RAG Agent Intelligence, [PRINCIPLE_6_NAME] → Localization & Translation Requirements
- Added sections: Content Architecture, Intelligence & Agent Guidelines, Language & Localization
- Removed sections: None
- Templates requiring updates: ✅ .specify/templates/plan-template.md, ✅ .specify/templates/spec-template.md, ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Physical AI & Humanoid Robotics Education
The primary mission is to create a unified educational resource that bridges digital AI (LLMs/Agents) and physical embodiment (Robots). All content, code examples, and learning materials must focus on practical implementation of robotics concepts with emphasis on modern AI integration. The system must serve as both a learning platform and practical reference for building embodied intelligence systems.

### Docusaurus Frontend Architecture
The frontend must be built using Docusaurus 3.x with React/TypeScript, completely replacing default Docusaurus classic styles with Tailwind CSS. The system must deploy to GitHub Pages with FastAPI backend (deployable to Vercel) for RAG agent functionality. All UI components must be responsive and optimized for both desktop and mobile learning experiences.

### Cyberpunk Industrial UI/UX
The visual identity must follow a "Cyberpunk/Industrial Robotics" theme with dark mode as default, using neon green (NVIDIA), electric blue (ROS 2), and warning orange (Hardware Safety) as accent colors. The landing page must feature high-tech imagery, 3D elements, or strong typographic hierarchy emphasizing "Embodied Intelligence" without using default Docusaurus banners.

### ROS 2 Python Code Standards
All robot control code must be Python-based using `rclpy` for ROS 2 Humble/Iron compatibility. Every code block must be syntactically correct and verified for the target ROS 2 distribution. Architecture diagrams must use Mermaid.js for ROS Nodes/Topics visualization. Code examples must follow ROS 2 best practices and be testable in both simulation and real hardware contexts.

### RAG Agent Intelligence
The embedded "Vector" agent must function as a "Lab Assistant" with RAG capabilities answering questions strictly based on book content. The agent must be context-aware of the current chapter, perform hardware compatibility checks (especially NVIDIA Isaac Sim RTX GPU requirements), and adapt responses based on user's authentication status and defined hardware background.

### Localization & Translation Requirements
The system must support multilingual content with primary English (Technical) delivery and mandatory Urdu translation toggle available at the start of every chapter. Translation functionality must be seamlessly integrated without disrupting the learning flow, and all new content must be designed with localization in mind from the initial development phase.

## Content Architecture
The book must follow the specific Hackathon syllabus with 4 Course Modules: Module 1 (The Robotic Nervous System - ROS 2, Nodes, rclpy, URDF), Module 2 (The Digital Twin - Gazebo Physics, Unity Rendering, Sensors), Module 3 (The AI-Robot Brain - NVIDIA Isaac Sim, VSLAM, Nav2), Module 4 (Vision-Language-Action - OpenAI Whisper, LLM Cognitive Planning), and Capstone (The Autonomous Humanoid). Navigation must be streamlined with a sidebar for these modules.

## Intelligence & Agent Guidelines
The embedded chatbot ("Vector" agent) acts as a "Lab Assistant" with strict RAG functionality based on book content only. It must be context-aware of user's current chapter, warn about hardware requirements (MacBook/Raspberry Pi users need RTX GPU for NVIDIA Isaac Sim), and personalize responses based on authenticated user's hardware background stored in Better-Auth. The agent must never hallucinate information outside the book content.

## Language & Localization
Primary language is English (Technical) with mandatory Urdu translation toggle at the start of every chapter. The translation system must maintain technical accuracy while being accessible to Urdu-speaking students. All new content additions must consider localization requirements from the initial planning phase.

## Governance
This constitution governs all development, design, and content decisions for the Physical AI & Humanoid Robotics Textbook project. All PRs and reviews must verify compliance with these principles. Any architectural changes affecting the core tech stack (Docusaurus, FastAPI, ROS 2, RAG agent) require explicit approval. The constitution must be referenced during all planning sessions to ensure alignment with the project mission.

**Version**: 1.1.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
