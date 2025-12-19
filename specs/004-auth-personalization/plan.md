# Implementation Plan: Secure Authentication and User Personalization

**Branch**: `004-auth-personalization` | **Date**: 2025-12-19 | **Spec**: [specs/004-auth-personalization/spec.md](spec.md)
**Input**: Feature specification from `/specs/004-auth-personalization/spec.md`

## Summary
Implement a complete authentication and personalization system using BetterAuth. This involves setting up an authentication server, a signup flow that captures user software/hardware background, and integrating this state into the Docusaurus frontend to provide tailored technical content.

## Technical Context

**Language/Version**: TypeScript/Node.js (for Auth), Python 3.10+ (for RAG Backend), React (Frontend)
**Primary Dependencies**: `better-auth`, `docusaurus-plugin-content-docs` (customization), `fastapi`
**Storage**: [NEEDS CLARIFICATION: Database choice for BetterAuth (e.g., SQLite, PostgreSQL, or a managed service like Supabase)]
**Testing**: `vitest` (Auth logic), `playwright` (End-to-end flows)
**Target Platform**: Vercel (Frontend), Hugging Face / Render (Backend)
**Project Type**: Full-stack (React Frontend + TS Auth + Python Data Pipeline)
**Performance Goals**: Auth handshake < 500ms, Content personalization overhead < 100ms
**Constraints**: Must adhere to "No external UI frameworks" (use native CSS/Docusaurus UI)
**Scale/Scope**: Support initial user base of panaversity students

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Principle I (Mission)**: Personalization directly supports the goal of guiding students based on their specific tools. (PASS)
- **Principle IV (Docusaurus Specific)**: Personalization will use the `<PersonalizationToggle />` and conditional MDX rendering as defined. (PASS)
- **Principle VII (Language)**: Auth error messages and labels will be in English. (PASS)

## Project Structure

### Documentation (this feature)

```text
specs/004-auth-personalization/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code

```text
AI-BOOK/                 # Frontend (Docusaurus)
├── src/
│   ├── components/
│   │   ├── Auth/        # Auth UI components (Signup, Signin)
│   │   └── Personalization/ # Tailored content wrapper
│   └── theme/           # Swizzled components for auth guards
└── ...

backend/                 # Existing Python Backend
└── ...

auth-server/             # [PROPOSED] New TypeScript Auth Server
├── src/
│   ├── index.ts         # BetterAuth configuration
│   └── db/              # Database adapter
└── package.json
```

**Structure Decision**: BetterAuth requires a TypeScript environment. Since the main book app is Docusaurus (React), we will either integrate BetterAuth directly into the Docusaurus build (if using a serverless provider) or create a small `auth-server` directory to handle identity management.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| New project folder (`auth-server`) | BetterAuth is TS-native and doesn't have a Python SDK for the FastAPI backend. | Integrating TS logic into Docusaurus's build process can be brittle for complex auth flows. |