# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `01-physical-ai-textbook` | **Date**: 2025-12-10 | **Spec**: [link](../specs/01-physical-ai-textbook/spec.md)
**Input**: Feature specification from `/specs/01-physical-ai-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The Physical AI & Humanoid Robotics Textbook project will be implemented as an AI-native Docusaurus website with a custom cyberpunk/industrial robotics theme. The implementation will follow a phased approach starting with the visual overhaul and foundation, followed by content architecture, the Vector RAG agent backend, and finally the integration and personalization features. The system will provide a comprehensive learning experience with interactive AI assistance and localization capabilities.

## Technical Context

**Language/Version**: TypeScript 5.x, Python 3.11+
**Primary Dependencies**: Docusaurus 3.x, React 18+, Tailwind CSS 3.x, FastAPI 0.104+, Qdrant 1.7+, Better-Auth 1.x
**Storage**: Neon (PostgreSQL) for user data, Qdrant (Vector DB) for content embeddings
**Testing**: Jest for frontend, pytest for backend, Playwright for E2E testing
**Target Platform**: Web application (GitHub Pages frontend, optional Vercel backend)
**Project Type**: Web application (frontend/backend split)
**Performance Goals**: <2s page load times, <1s RAG response times, 99% uptime during peak hours
**Constraints**: Must support RTX GPU compatibility checks, Urdu localization, ROS 2 Humble/Iron compatibility
**Scale/Scope**: Target 10k+ students, 100+ content pages, 1M+ embeddings in vector DB

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Required Compliance Checks:
- **Frontend Architecture**: Verify implementation uses Docusaurus 3.x with React/TypeScript and Tailwind CSS (no default Docusaurus classic styles)
- **UI/UX Compliance**: Confirm cyberpunk/industrial robotics theme with dark mode default and specified accent colors (neon green, electric blue, warning orange)
- **Tech Stack Alignment**: Validate FastAPI backend with Neon (PostgreSQL) and Qdrant (Vector DB) integration
- **Authentication**: Ensure Better-Auth implementation for user personalization
- **ROS 2 Code Standards**: Verify all robot control code uses Python `rclpy` for ROS 2 Humble/Iron compatibility
- **Content Structure**: Confirm alignment with 4-module syllabus (Modules 1-4 + Capstone)
- **RAG Agent**: Verify Vector agent implementation with context-awareness and hardware compatibility checks
- **Localization**: Confirm Urdu translation toggle available at chapter start
- **Deployment**: Validate GitHub Pages frontend with optional Vercel API deployment

## Project Structure

### Documentation (this feature)

```text
specs/01-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application (when "frontend" + "backend" detected)
frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   ├── TranslationToggle/
│   │   └── PersonalizationPrompt/
│   ├── pages/
│   │   └── index.tsx
│   ├── css/
│   │   └── custom.css
│   └── services/
│       └── api.ts
├── docs/
│   ├── 01-module-1/
│   ├── 02-module-2/
│   ├── 03-module-3/
│   └── 04-module-4/
├── static/
└── docusaurus.config.js

backend/
├── src/
│   ├── models/
│   ├── services/
│   │   ├── rag_search.py
│   │   ├── hardware_check.py
│   │   └── ros_coder.py
│   ├── api/
│   │   └── main.py
│   └── utils/
├── requirements.txt
└── config.py

# Configuration files
sidebars.ts
tailwind.config.js
postcss.config.js
```

**Structure Decision**: Web application with Docusaurus frontend and FastAPI backend, following the constitution's requirement for GitHub Pages deployment with optional Vercel backend for RAG agent functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |