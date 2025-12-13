# Docusaurus Sci-Fi UI Redesign Tasks

## Overview
Redesign the default Docusaurus UI into a futuristic, sci-fi, AI-native interface for a Physical AI & Humanoid Robotics textbook. This task is VISUAL ONLY. No backend logic. No authentication logic. No chatbot logic. No translation logic.

## Global Design Language
- Dark space-black background
- Neon cyan + violet accents
- Glassmorphism panels
- Subtle HUD grid lines
- Academic, research-grade sci-fi (not flashy, not marketing)

## Constraints
- Use native Docusaurus theming
- No external UI frameworks
- No marketing visuals
- Must feel like an AI research terminal

## Phases

### Phase 1: Setup (Project Initialization)

- [x] T001 Initialize Docusaurus project (e.g., `npx create-docusaurus@latest my-website classic --typescript` and navigate into `my-website` directory)

### Phase 2: Foundational - Global Design Language

- [x] T002 Configure `docusaurus.config.js` for dark theme and custom CSS in `docusaurus.config.js`
- [x] T003 Define global CSS variables for colors (space-black, neon cyan, violet) and fonts in `src/css/custom.css`
- [x] T004 Implement glassmorphism base styles (background blur, transparency) for panels and cards in `src/css/custom.css`
- [x] T005 Add subtle HUD grid lines using CSS to the main layout in `src/css/custom.css`

### Phase 3: User Story 1: Navbar (Spaceship Control Panel)

**Story Goal**: Create a Navbar that feels like a spaceship control panel.
**Independent Test Criteria**: Navbar visually matches the sci-fi theme with all specified elements and styling.

- [x] T006 [US1] Override Docusaurus Navbar component for custom layout in `src/theme/Navbar/index.js`
- [x] T007 [US1] Implement "Physical AI" project title (left) in `src/theme/Navbar/index.js`
- [x] T008 [US1] Add "Textbook", "Labs", "Capstone", "Reference" center menu items in `src/theme/Navbar/index.js`
- [x] T009 [US1] Implement "اردو" toggle (visual only, right) in `src/theme/Navbar/index.js`
- [x] T010 [US1] Add static "Login" and "Signup" buttons (visual only, right) in `src/theme/Navbar/index.js`
- [x] T011 [US1] Apply sci-fi styling (colors, glassmorphism, neon accents) to Navbar components in `src/theme/Navbar/styles.module.css` and `src/css/custom.css`

### Phase 4: User Story 2: Landing Page - Hero Section

**Story Goal**: Design a futuristic hero section for the landing page.
**Independent Test Criteria**: Hero section displays title, subtitle, and background effects as specified.

- [x] T012 [US2] Create or modify `src/pages/index.js` for the custom landing page layout
- [x] T013 [US2] Implement large title: “PHYSICAL AI” in `src/pages/index.js`
- [x] T014 [US2] Implement subtitle: “Embodied Intelligence in the Physical World” in `src/pages/index.js`
- [x] T015 [US2] Add futuristic background grid / glow effects using CSS in `src/pages/index.module.css` or `src/css/custom.css`

### Phase 5: User Story 3: Landing Page - Modules Section

**Story Goal**: Create a card-based modules section where each card looks like a system panel.
**Independent Test Criteria**: Modules section displays all specified module cards with system panel styling and hover effects.

- [x] T016 [US3] Create a new React component `src/components/ModuleCard/index.js` for individual module cards
- [x] T017 [US3] Implement card-based layout for the Modules section in `src/pages/index.js`
- [x] T018 [US3] Populate modules: "ROS 2 Nervous System", "Digital Twin Simulation", "NVIDIA Isaac AI", "Vision-Language-Action", "Capstone Project" using `ModuleCard` component in `src/pages/index.js`
- [x] T019 [US3] Style `ModuleCard` as a system panel with subtle hover glow effect in `src/components/ModuleCard/styles.module.css`

### Phase 6: User Story 4: Landing Page - Skills Section

**Story Goal**: Display AI-Native Skills using sci-fi styled cards.
**Independent Test Criteria**: Skills section displays all specified skill cards with appropriate styling.

- [x] T020 [US4] Implement title: “AI-Native Skills” in `src/pages/index.js`
- [x] T021 [US4] Create a new React component `src/components/SkillCard/index.js` for individual skill cards
- [x] T022 [US4] Implement card layout for the Skills section using `SkillCard` in `src/pages/index.js`
- [x] T023 [US4] Populate skills: "Robotics Systems Thinking", "Physical AI Design", "Simulation & Digital Twins", "Vision-Language-Action Reasoning" using `SkillCard` in `src/pages/index.js`
- [x] T024 [US4] Apply sci-fi styling to `SkillCard` in `src/components/SkillCard/styles.module.css`

### Phase 7: User Story 5: Landing Page - Agents Section

**Story Goal**: Showcase intelligent agents in an AI lab dashboard style.
**Independent Test Criteria**: Agents section displays all specified agent cards with AI lab dashboard styling.

- [x] T025 [US5] Implement title: “Intelligent Agents” in `src/pages/index.js`
- [x] T026 [US5] Create a new React component `src/components/AgentCard/index.js` for individual agent cards
- [x] T027 [US5] Implement card layout for the Agents section using `AgentCard` in `src/pages/index.js`
- [x] T028 [US5] Populate agents: "Textbook Architect", "Physical AI Professor", "Futuristic UI Designer" with roles in one line, using `AgentCard` in `src/pages/index.js`
- [x] T029 [US5] Apply AI lab dashboard styling to `AgentCard` in `src/components/AgentCard/styles.module.css`

### Phase 8: User Story 6: Chatbot UI

**Story Goal**: Add a floating, circular/hex sci-fi styled chatbot button.
**Independent Test Criteria**: Floating chatbot button is present, correctly positioned, and styled as specified.

- [x] T030 [US6] Create a new React component `src/components/FloatingChatbotButton/index.js`
- [x] T031 [US6] Implement floating button positioned bottom-right on all pages (e.g., by wrapping `Layout` or in `docusaurus.config.js` custom fields and using `DocPaginator`) in `src/components/FloatingChatbotButton/index.js`
- [x] T032 [US6] Style as circular / hex sci-fi with "AI Assistant" label in `src/components/FloatingChatbotButton/styles.module.css`

### Phase 9: User Story 7: Docs UI

**Story Goal**: Redesign the documentation sidebar and content area for an immersive reading experience.
**Independent Test Criteria**: Docs sidebar and content area are styled according to the sci-fi theme, optimized for readability.

- [x] T033 [US7] Override Docusaurus `DocSidebar` component for custom styling in `src/theme/DocSidebar/index.js`
- [x] T034 [US7] Apply glass panel effect to `DocSidebar` in `src/theme/DocSidebar/styles.module.css`
- [x] T035 [US7] Implement active item glow in `DocSidebar` in `src/theme/DocSidebar/styles.module.css`
- [x] T036 [US7] Override Docusaurus `DocItem` component for content area styling in `src/theme/DocItem/index.js`
- [x] T037 [US7] Optimize `DocItem` content area for long reading sessions (font, line-height, spacing) in `src/theme/DocItem/styles.module.css`

### Final Phase: Polish & Cross-Cutting Concerns

**Story Goal**: Ensure overall quality, responsiveness, and adherence to the design language.
**Independent Test Criteria**: UI is responsive, animations are subtle, and accessibility standards are met.

- [ ] T038 Ensure overall responsiveness across different screen sizes using CSS media queries in `src/css/custom.css`
- [ ] T039 Review and refine all animations and transitions for a subtle, futuristic feel across the UI in relevant CSS files
- [ ] T040 Perform accessibility audit for contrast, focus states, and readability of all text elements
- [ ] T041 Clean up unused CSS rules and components for optimal performance

## Dependencies
The phases are ordered sequentially. Within each phase, tasks can often be parallelized, especially within UI component creation and styling.

## Parallel Execution Examples
- **Phase 2 (Foundational)**: T003, T004, T005 can be worked on in parallel once T002 is complete.
- **Phase 3 (Navbar)**: T007, T008, T009, T010 can be developed in parallel once T006 is in progress.
- **Phase 5 (Modules Section)**: T016, T017, T018, T019 can be developed in parallel once T012 is in progress.

## Implementation Strategy
Start with an MVP (Minimum Viable Product) focusing on Phase 1, Phase 2, and then the Navbar (Phase 3) and Hero Section (Phase 4). Subsequent sections of the landing page (Modules, Skills, Agents), Chatbot UI, and Docs UI can be incrementally delivered.

