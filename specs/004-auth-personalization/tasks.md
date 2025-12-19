---
description: "Task list for Secure Authentication and User Personalization"
---

# Tasks: Secure Authentication and User Personalization

**Input**: Design documents from `specs/004-auth-personalization/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/auth-api.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: [US1], [US2], [US3] (maps to user stories from spec.md)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create `auth-server/` directory and initialize `npm` project
- [x] T002 Install `hono`, `better-auth`, and `better-sqlite3` in `auth-server/`
- [x] T003 [P] Create `.env` template in `auth-server/.env.example` with `BETTER_AUTH_SECRET` and `DATABASE_URL`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core auth infrastructure

- [x] T004 Configure BetterAuth instance with SQLite adapter in `auth-server/src/auth.ts`
- [x] T005 Extend User schema with custom fields (`operating_system`, `experience_level`, `hardware_access`) in `auth-server/src/auth.ts`
- [x] T006 Implement Hono server with BetterAuth handler in `auth-server/src/index.ts`
- [x] T007 [P] Create `AuthContext` and hooks in `AI-BOOK/src/components/Auth/AuthContext.tsx`

**Checkpoint**: Auth server is running and can handle basic session requests.

---

## Phase 3: User Story 1 & 2 - Signup & Background Collection (Priority: P1) 🎯 MVP

**Goal**: Implement secure signup that captures user background data.

**Independent Test**: Complete the signup form and verify user record in `sqlite.db` contains background data.

### Implementation for US1 & US2

- [x] T008 [P] [US1] Create Signup UI component with form validation in `AI-BOOK/src/components/Auth/Signup.tsx`
- [x] T009 [P] [US1] Create Signin UI component in `AI-BOOK/src/components/Auth/Login.tsx`
- [x] T010 [US2] Add background question fields (OS, Level, Hardware) to `Signup.tsx`
- [x] T011 [US1] Implement signup/signin logic using `better-auth` client in `AuthContext.tsx`
- [x] T012 [US1] Integrate Signup/Signin components into Docusaurus pages (e.g., `/signup`, `/login`)

**Checkpoint**: Users can create accounts with background data and log in.

---

## Phase 4: User Story 3 - Personalized Content Delivery (Priority: P2)

**Goal**: Use auth state to tailor MDX content.

**Independent Test**: Log in as a "Windows" user and see only Windows instructions in a test doc.

### Implementation for US3

- [x] T013 [US3] Wrap Docusaurus application with `AuthProvider` in `AI-BOOK/src/theme/Root.tsx`
- [x] T014 [P] [US3] Create `PersonalizedContent` wrapper component in `AI-BOOK/src/components/Personalization/index.tsx`
- [x] T015 [US3] Implement logic in `PersonalizedContent` to check `AuthContext` and conditionally render children
- [x] T016 [US3] Add a "Personalization Settings" page to update background data in `AI-BOOK/src/pages/profile.tsx`

**Checkpoint**: Content dynamically reacts to logged-in user's profile.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final verification and documentation.

- [x] T017 Implement basic error handling and loading states for auth forms
- [x] T018 Update `specs/004-auth-personalization/quickstart.md` with final deployment steps
- [x] T019 [P] Document the authentication flow and schema extension in `specs/004-auth-personalization/architecture.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Phase 1.
- **Signup Implementation (Phase 3)**: Depends on Phase 2.
- **Personalization (Phase 4)**: Depends on Phase 3 (needs active session).

### Parallel Opportunities

- Frontend UI components (T008, T009) can be built while backend (T004, T005) is being configured.
- `AuthContext` (T007) and `auth-server` (T006) can be developed concurrently once the schema is defined.
