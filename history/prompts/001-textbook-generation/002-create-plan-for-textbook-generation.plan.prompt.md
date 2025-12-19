---
id: 002
title: Create Plan for Textbook Generation
stage: plan
date: 2025-12-15
surface: agent
model: gemini-1.5-pro
feature: 001-textbook-generation
branch: 001-textbook-generation
user: '[USER]'
command: /sp.plan
labels: [plan, design, documentation]
links:
  spec: specs/001-textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-textbook-generation/plan.md
- specs/001-textbook-generation/data-model.md
- specs/001-textbook-generation/quickstart.md
- GEMINI.md
tests:
- "none"
---

## Prompt

---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType gemini`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

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


/sp.plan name: "Physical_AI_Book_Execution_Plan"
strategy: "Sequential_Dependency"

phases:
  # ==================================================
  # PHASE 1: SCAFFOLDING & FOUNDATIONS
  # Goal: Establish the environment and basic theory.
  # ==================================================
  - name: "Phase 1: The Digital Foundation"
    tasks:
      - "write_intro"             # From sp.spec
      - "write_hardware_setup"    # From sp.spec
    validation:
      - "Check if 'docs/intro.mdx' exists."
      - "Ensure hardware warnings (:::danger) are present in setup guide."

  # ==================================================
  # PHASE 2: THE NERVOUS SYSTEM (ROS 2)
  # Goal: Get the user coding nodes.
  # ==================================================
  - name: "Phase 2: ROS 2 Architecture"
    dependencies: ["Phase 1"]
    tasks:
      - "module_1_ros2_concepts"
      - "module_1_ros2_code"
    validation:
      - "Verify Python code blocks use valid 'rclpy' syntax."
      - "Ensure <PersonalizedContent> tags are inserted."

  # ==================================================
  # PHASE 3: THE DIGITAL TWIN (SIMULATION)
  # Goal: Physics and Environment.
  # ==================================================
  - name: "Phase 3: Simulation & Physics"
    dependencies: ["Phase 2"]
    tasks:
      - "module_2_urdf"
      - "module_2_gazebo"
    validation:
      - "Ensure URDF vs SDF distinction is clear."
      - "Check for Gazebo Real-Time Factor warnings."

  # ==================================================
  # PHASE 4: EMBODIED INTELLIGENCE (ISAAC & VLA)
  # Goal: Advanced AI integration.
  # ==================================================
  - name: "Phase 4: The AI Brain"
    dependencies: ["Phase 3"]
    tasks:
      - "module_3_isaac_sim"
      - "module_3_nav2"
      - "module_4_vla_theory"
      - "module_4_whisper"
    validation:
      - "Check for OpenAI Whisper API integration code."
      - "Ensure Nav2 instructions match the selected hardware from sp.clarify."

  # ==================================================
  # PHASE 5: POST-PROCESSING (BONUS POINTS)
  # Goal: Optimization and Formatting.
  # ==================================================
  - name: "Phase 5: Hackathon Compliance Check"
    tasks:
      - action: "scan_formatting"
        rule: "Ensure all frontmatter has sidebar_label."
      - action: "insert_components"
        rule: "Ensure <TranslationButton /> and <ChatBotContext /> exists in all MDX files."

## Response snapshot

I have completed the implementation plan for 'Physical AI Textbook Generation'.
- **Branch:** `001-textbook-generation`
- **Plan File:** `specs/001-textbook-generation/plan.md`
- **Generated Artifacts:**
    - `specs/001-textbook-generation/data-model.md`
    - `specs/001-textbook-generation/quickstart.md`
- **Agent Context:** The agent context has been updated with the technologies from the plan.

The project is now ready for the task generation phase, which would be `/sp.tasks`.

## Outcome

- ✅ Impact: A new implementation plan has been created, outlining the technical context, constitutional gates, and design artifacts.
- 🧪 Tests: No tests were run.
- 📁 Files: 
  - `specs/001-textbook-generation/plan.md`
  - `specs/001-textbook-generation/data-model.md`
  - `specs/001-textbook-generation/quickstart.md`
  - `GEMINI.md`
- 🔁 Next prompts: The user can now proceed with the task generation phase for this feature using `/sp.tasks`.
- 🧠 Reflection: The process of reading the spec and constitution to generate a plan worked well. The plan provides a clear path forward for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
