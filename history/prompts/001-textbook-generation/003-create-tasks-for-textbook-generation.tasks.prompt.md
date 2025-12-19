---
id: 003
title: Create Tasks for Textbook Generation
stage: tasks
date: 2025-12-15
surface: agent
model: gemini-1.5-pro
feature: 001-textbook-generation
branch: 001-textbook-generation
user: '[USER]'
command: /sp.tasks
labels: [tasks, planning, documentation]
links:
  spec: specs/001-textbook-generation/spec.md
  plan: specs/001-textbook-generation/plan.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-textbook-generation/tasks.md
tests:
- "none"
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: tasks: # ================================================== # MODULE 1: THE ROBOTIC NERVOUS SYSTEM (ROS 2) # Focus: Nodes, Topics, Rclpy, Environment Setup # ================================================== - id: "MOD1-01" file_path: "docs/02-module-1-ros2/01-introduction.mdx" assigned_to: "content_writer" priority: "High" instructions: |
**Prompt for Agent:** Write a comprehensive Introduction for 'Module 1: The Robotic Nervous System'.
**Key Topics to Cover:**
1. **Physical AI vs Generative AI:** Explain that Physical AI requires 'Embodied Intelligence' (understanding physics, gravity, and real-world constraints), whereas GenAI (ChatGPT) only understands text/images.
2. **What is ROS 2?** Define Robot Operating System as the "middleware" or "glue" that connects sensors (eyes), actuators (muscles), and algorithms (brain).
3. **Why Linux?** Briefly explain why Ubuntu 22.04 is the industry standard for robotics (real-time scheduling, open-source).
4. **The Goal:** End the chapter by stating that by the end of this module, the student will control a simulated robot's nervous system.
**Formatting:**
- Use ':::' for notes.
- Keep tone inspiring but technical.
- id: "MOD1-02" file_path: "docs/02-module-1-ros2/02-environment-setup.mdx" assigned_to: "lab_instructor" priority: "Critical" instructions: |
**Prompt for Agent:** Write a technical setup guide for 'Installing ROS 2 Humble on Ubuntu 22.04'.
**Steps to Include:**
1. **Locale Setup:** Commands to set UTF-8.
2. **Add Repositories:** Adding the ROS 2 GPG key and repository to `apt`.
3. **Installation:** Command: `sudo apt install ros-humble-desktop`.
4. **Environment Setup:** Crucial step: `source /opt/ros/humble/setup.bash`. Use a `:::danger` admonition to warn users that if they forget this, ROS commands won't work.
5. **Verification:** Show the `ros2 run demo_nodes_cpp talker` and `listener` example.
**Constraint:**
- Strictly use ROS 2 Humble commands. Do NOT show ROS 1 (Melodic/Noetic) commands.
- id: "MOD1-03" file_path: "docs/02-module-1-ros2/03-architecture.mdx" assigned_to: "content_writer" priority: "High" instructions: |
**Prompt for Agent:** Explain the Core Architecture of ROS 2 using the 'Nervous System' analogy.
**Concepts to Define:**
1. **Nodes (The Neurons):** Single-purpose executable programs (e.g., CameraDriver, MotorController).
2. **Topics (The Nerves):** Explain the Publish/Subscribe model. Use an analogy like "YouTube Channel" (Publisher) and "Subscriber".
3. **Messages:** The data packets sent over topics.
4. **Services:** Explain the Request/Response pattern (Synchronous), different from Topics (Asynchronous).
**Formatting:**
- Use diagrams descriptions (e.g., [Node A] -> /topic -> [Node B]).
- Use `:::tip` to explain when to use Topics vs Services.
- id: "MOD1-04" file_path: "docs/02-module-1-ros2/04-python-nodes.mdx" assigned_to: "lab_instructor" priority: "Critical" instructions: |
**Prompt for Agent:** Write a coding tutorial: 'Writing Your First ROS 2 Node in Python'.
**Code Requirements:**
1. **Workspace:** Show `mkdir -p ros2_ws/src` and `colcon build`.
2. **Publisher Node:** Write a complete Python script using `rclpy` that publishes a string "Hello Physical AI" every 0.5 seconds.
3. **Subscriber Node:** Write a script that listens to that topic and prints the data.
4. **Explanation:** Comment the code heavily. Explain `create_timer`, `create_subscription`, and `spin`.
5. **Execution:** Show how to run `python3 publisher.py` (or via `ros2 run`).
**Tech Stack:**
- Python 3.10+
- rclpy library
- id: "MOD1-05" file_path: "docs/02-module-1-ros2/05-launch-files.mdx" assigned_to: "lab_instructor" priority: "Medium" instructions: |
**Prompt for Agent:** Explain and demonstrate 'ROS 2 Launch Files'.
**Content:**
1. **The Problem:** Explain that running 20 nodes in 20 terminals is impossible.
2. **The Solution:** Launch files (Python-based in ROS 2).
3. **Code Example:** Create a file named `bringup_robot.launch.py`.
4. **Logic:** Show how to import `Node` from `launch_ros.actions` and return a `LaunchDescription` that starts both the Publisher and Subscriber from the previous lesson simultaneously.
**Note:**
- Emphasize that ROS 2 uses Python for launch files, unlike ROS 1 which used XML.

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

I have generated the task list for the 'Physical AI Textbook Generation' feature.
- **Tasks File:** `specs/001-textbook-generation/tasks.md`
- **Total Tasks:** 13
- **MVP Scope (User Story 1):** 2 tasks (Introduction and Hardware Setup)
- **Parallel Opportunities:** Most tasks can be run in parallel within their respective user story phases.

The project is now ready for implementation. You can start the implementation by running `/sp.implement`.

## Outcome

- ✅ Impact: A detailed, actionable task list has been created, breaking down the feature implementation into manageable, dependency-ordered steps.
- 🧪 Tests: No tests were run.
- 📁 Files: 
  - `specs/001-textbook-generation/tasks.md`
- 🔁 Next prompts: The user can now proceed with the implementation phase for this feature using `/sp.implement`.
- 🧠 Reflection: The process of mapping user stories from the spec to a detailed task list worked well. The generated `tasks.md` provides a clear roadmap for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
