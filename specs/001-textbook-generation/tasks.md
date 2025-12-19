# Tasks: Physical AI Textbook Generation

**Input**: Design documents from `/specs/001-textbook-generation/`

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: User story this task belongs to (US1, US2, etc.)

---

## Phase 1: Setup (User Story 1)

**Goal**: Establish the environment and basic theory.
**Independent Test**: The introduction and hardware setup guides are generated and correct.

- [x] T001 [US1] [P] Generate Introduction for 'Module 1: The Robotic Nervous System' in `docs/02-module-1-ros2/01-introduction.mdx`
- [x] T002 [US1] [P] Generate technical setup guide for 'Installing ROS 2 Humble on Ubuntu 22.04' in `docs/02-module-1-ros2/02-environment-setup.mdx`

---

## Phase 2: Foundational (User Story 2)

**Goal**: Get the user coding ROS 2 nodes.
**Independent Test**: The ROS 2 architecture explanation and Python node tutorials are generated and the code is valid.

- [x] T003 [US2] [P] Explain the Core Architecture of ROS 2 in `docs/02-module-1-ros2/03-architecture.mdx`
- [x] T004 [US2] Write a coding tutorial: 'Writing Your First ROS 2 Node in Python' in `docs/02-module-1-ros2/04-python-nodes.mdx`
- [x] T005 [US2] Explain and demonstrate 'ROS 2 Launch Files' in `docs/02-module-1-ros2/05-launch-files.mdx` (depends on T004)

---

## Phase 3: User Story 3 - Simulating a Robot

**Goal**: Teach the user about robot simulation with URDF and Gazebo.
**Independent Test**: The URDF guide and Gazebo simulation tutorial are generated.

- [x] T006 [US3] [P] Write an introduction to "The Digital Twin" in `docs/03-module-2-simulation/01-digital-twin-concept.mdx`
- [x] T007 [US3] [P] Explain URDF (Unified Robot Description Format) in `docs/03-module-2-simulation/02-understanding-urdf.mdx`
- [x] T008 [US3] [P] Tutorial on "Simulating Physics in Gazebo" in `docs/03-module-2-simulation/03-gazebo-setup.mdx`
- [x] T009 [US3] [P] Guide on adding a LiDAR or Camera to the robot in `docs/03-module-2-simulation/04-adding-sensors.mdx`

---

## Phase 4: User Story 4 - The AI Brain (NVIDIA Isaac)

**Goal**: Introduce advanced AI-driven simulation and navigation.
**Independent Test**: The Isaac Sim introduction and Nav2 tutorials are generated.

- [x] T010 [US4] [P] Write 'Introduction to NVIDIA Isaac Sim' in `docs/04-module-3-isaac/intro-isaac-sim.mdx`
- [x] T011 [US4] [P] Write 'Navigation & VSLAM with Isaac ROS' in `docs/04-module-3-isaac/nav2-vslam.mdx`

---

## Phase 5: User Story 5 - Vision Language Action (VLA)

**Goal**: Implement voice control and connect LLMs to the robot.
**Independent Test**: The VLA theory and Whisper integration tutorials are generated.

- [x] T012 [US5] [P] Write 'Vision-Language-Action (VLA) Models' in `docs/05-module-4-vla/vla-theory.mdx`
- [x] T013 [US5] [P] Write 'Voice Control with OpenAI Whisper' in `docs/05-module-4-vla/voice-control.mdx`

---

## Phase N: Polish & Cross-Cutting Concerns

- [x] T014 [P] Scan all generated files for formatting and consistency with the constitution.
- [x] T015 [P] Verify all interactive components (`<ChatBotContext>`, `<PersonalizedContent>`) are present where required.

---

## Dependencies & Execution Order

- **Phase 1 (US1)** can start immediately.
- **Phase 2 (US2)** can start after Phase 1.
- **Phase 3 (US3)** can start after Phase 2.
- **Phase 4 (US4)** can start after Phase 3.
- **Phase 5 (US5)** can start after Phase 4.
- **Phase N (Polish)** can start after all other phases are complete.

Within each phase, tasks marked `[P]` can be run in parallel.