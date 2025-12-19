---
id: 001
title: Create Spec for Textbook Generation
stage: spec
date: 2025-12-15
surface: agent
model: gemini-1.5-pro
feature: 001-textbook-generation
branch: 001-textbook-generation
user: '[USER]'
command: /sp.specify
labels: [spec, planning, documentation]
links:
  spec: specs/001-textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-textbook-generation/spec.md
- specs/001-textbook-generation/checklists/requirements.md
tests:
- "none"
---

## Prompt

---
description: Create or update the feature specification from a natural language feature description.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.specify` in the triggering message **is** the feature description. Assume you always have it available in this conversation even if `name: "Physical_AI_Textbook_Generation" description: "Generates the complete Physical AI & Humanoid Robotics textbook." version: "1.0" # Global context provided to all tasks context: constitution: "sp.constitution" project_name: "Physical AI & Humanoid Robotics" author: "Mehma Rashid" tasks: # ========================================== # INTRODUCTION & ONBOARDING # ========================================== - id: "write_intro" agent: "content_writer" description: "Write the course landing page and introduction." prompt: | Write the 'Introduction to Physical AI'. - Define 'Embodied Intelligence'. - Explain why we are moving from Digital Agents (Chatbots) to Physical Agents (Robots). - Include a section on the 'Startup Opportunity' in robotics. - Add a <ChatBotContext topic="introduction" /> component at the bottom. output_file: "docs/intro.mdx" - id: "write_hardware_setup" agent: "content_writer" description: "Detailed hardware guide with safety warnings." prompt: | Write the 'Hardware & Lab Setup' guide. - Compare 'Digital Twin Workstation' (RTX 4070 Ti+) vs 'Edge Kit' (Jetson Orin Nano). - Detail the 'Robot Lab' options: Proxy (Unitree Go2) vs Humanoid (Unitree G1). - Use :::danger to warn about LiPo battery handling and VRAM limitations. - Use :::tip for suggesting Cloud Workstations (AWS g5.2xlarge) if local GPU is unavailable. output_file: "docs/01-setup/hardware-requirements.mdx" # ========================================== # MODULE 1: THE ROBOTIC NERVOUS SYSTEM (ROS 2) # ========================================== - id: "module_1_ros2_concepts" agent: "content_writer" description: "Core concepts of ROS 2." prompt: | Write 'Module 1: ROS 2 Architecture'. - Explain Nodes, Topics, Services, and Actions using a 'Nervous System' analogy. - Explain why we use ROS 2 Humble/Iron and Linux (Ubuntu 22.04). - Include a <PersonalizedContent> block that explains ROS differently for Python Developers vs C++ Developers. output_file: "docs/02-module-1-ros2/ros2-architecture.mdx" - id: "module_1_ros2_code" agent: "lab_instructor" description: "Hands-on coding for ROS 2." prompt: | Write a tutorial: 'Your First ROS 2 Node'. - Create a Python (rclpy) Publisher node that simulates a 'Joint State' sensor. - Create a Subscriber node that reads this data. - Provide the full code in ```python blocks. - Explain the 'package.xml' and 'setup.py' configuration. output_file: "docs/02-module-1-ros2/first-node-python.mdx" # ========================================== # MODULE 2: DIGITAL TWIN (SIMULATION) # ========================================== - id: "module_2_urdf" agent: "content_writer" description: "Robot description formats." prompt: | Write 'Understanding URDF & SDF'. - Explain how we define a robot's body (Links) and muscles (Joints) in XML. - Explain the difference between URDF (standard) and SDF (Gazebo specific). - Show a code snippet of a simple URDF for a robotic leg. output_file: "docs/03-module-2-simulation/urdf-guide.mdx" - id: "module_2_gazebo" agent: "lab_instructor" description: "Setting up the simulation environment." prompt: | Write 'Simulating Physics in Gazebo'. - Guide the user on spawning the URDF into Gazebo. - Explain how to add Simulated Sensors (Lidar/Camera). - Use :::warning to check for 'Real Time Factor' drops if the CPU is weak. output_file: "docs/03-module-2-simulation/gazebo-physics.mdx" # ========================================== # MODULE 3: THE AI BRAIN (NVIDIA ISAAC) # ========================================== - id: "module_3_isaac_sim" agent: "content_writer" description: "Introduction to NVIDIA Omniverse and Isaac Sim." prompt: | Write 'Introduction to NVIDIA Isaac Sim'. - Explain the 'Omniverse' ecosystem and USD (Universal Scene Description) files. - Detail how Isaac Sim differs from Gazebo (Photorealism vs Pure Physics). - Explain 'Synthetic Data Generation' for training vision models. output_file: "docs/04-module-3-isaac/intro-isaac-sim.mdx" - id: "module_3_nav2" agent: "lab_instructor" description: "Navigation and SLAM." prompt: | Write 'Navigation & VSLAM with Isaac ROS'. - Explain how a robot maps a room using SLAM (Simultaneous Localization and Mapping). - Guide: Configuring 'Nav2' for a humanoid robot to walk from Point A to Point B avoiding obstacles. - Mention using the Intel RealSense D435i for depth data. output_file: "docs/04-module-3-isaac/nav2-vslam.mdx" # ========================================== # MODULE 4: VLA (VISION LANGUAGE ACTION) # ========================================== - id: "module_4_vla_theory" agent: "content_writer" description: "Theory of LLMs in Robotics." prompt: | Write 'Vision-Language-Action (VLA) Models'. - Explain how we connect LLMs (like GPT-4 or Gemini) to ROS 2 actions. - Concept: Translating "Clean the kitchen" -> [Navigate(Kitchen), Identify(Trash), Pick(Trash)]. - Introduce the 'Capstone Project: The Autonomous Humanoid'. output_file: "docs/05-module-4-vla/vla-theory.mdx" - id: "module_4_whisper" agent: "lab_instructor" description: "Voice control implementation." prompt: | Write 'Voice Control with OpenAI Whisper'. - Tutorial: Integrate OpenAI Whisper API to capture voice commands. - Code: A Python script that takes audio input and publishes a String to a ROS 2 topic `/voice_command`. - Add a <TranslationButton /> component placeholder for Urdu translation here. output_file: "docs/05-module-4-vla/voice-control.mdx"` appears literally below. Do not ask the user to repeat it unless they provided an empty command.

Given that feature description, do this:

1. **Generate a concise short name** (2-4 words) for the branch:
   - Analyze the feature description and extract the most meaningful keywords
   - Create a 2-4 word short name that captures the essence of the feature
   - Use action-noun format when possible (e.g., "add-user-auth", "fix-payment-bug")
   - Preserve technical terms and acronyms (OAuth2, API, JWT, etc.)
   - Keep it concise but descriptive enough to understand the feature at a glance
   - Examples:
     - "I want to add user authentication" → "user-auth"
     - "Implement OAuth2 integration for the API" → "oauth2-api-integration"
     - "Create a dashboard for analytics" → "analytics-dashboard"
     - "Fix payment processing timeout bug" → "fix-payment-timeout"

2. **Check for existing branches before creating new one**:
   
   a. First, fetch all remote branches to ensure we have the latest information:
      ```bash
      git fetch --all --prune
      ```
   
   b. Find the highest feature number across all sources for the short-name:
      - Remote branches: `git ls-remote --heads origin | grep -E 'refs/heads/[0-9]+-<short-name>$'`
      - Local branches: `git branch | grep -E '^[* ]*[0-9]+-<short-name>$'`
      - Specs directories: Check for directories matching `specs/[0-9]+-<short-name>`
   
   c. Determine the next available number:
      - Extract all numbers from all three sources
      - Find the highest number N
      - Use N+1 for the new branch number
   
   d. Run the script `.specify/scripts/powershell/create-new-feature.ps1 -Json "name: "Physical_AI_Textbook_Generation" description: "Generates the complete Physical AI & Humanoid Robotics textbook." version: "1.0" # Global context provided to all tasks context: constitution: "sp.constitution" project_name: "Physical AI & Humanoid Robotics" author: "Mehma Rashid" tasks: # ========================================== # INTRODUCTION & ONBOARDING # ========================================== - id: "write_intro" agent: "content_writer" description: "Write the course landing page and introduction." prompt: | Write the 'Introduction to Physical AI'. - Define 'Embodied Intelligence'. - Explain why we are moving from Digital Agents (Chatbots) to Physical Agents (Robots). - Include a section on the 'Startup Opportunity' in robotics. - Add a <ChatBotContext topic="introduction" /> component at the bottom. output_file: "docs/intro.mdx" - id: "write_hardware_setup" agent: "content_writer" description: "Detailed hardware guide with safety warnings." prompt: | Write the 'Hardware & Lab Setup' guide. - Compare 'Digital Twin Workstation' (RTX 4070 Ti+) vs 'Edge Kit' (Jetson Orin Nano). - Detail the 'Robot Lab' options: Proxy (Unitree Go2) vs Humanoid (Unitree G1). - Use :::danger to warn about LiPo battery handling and VRAM limitations. - Use :::tip for suggesting Cloud Workstations (AWS g5.2xlarge) if local GPU is unavailable. output_file: "docs/01-setup/hardware-requirements.mdx" # ========================================== # MODULE 1: THE ROBOTIC NERVOUS SYSTEM (ROS 2) # ========================================== - id: "module_1_ros2_concepts" agent: "content_writer" description: "Core concepts of ROS 2." prompt: | Write 'Module 1: ROS 2 Architecture'. - Explain Nodes, Topics, Services, and Actions using a 'Nervous System' analogy. - Explain why we use ROS 2 Humble/Iron and Linux (Ubuntu 22.04). - Include a <PersonalizedContent> block that explains ROS differently for Python Developers vs C++ Developers. output_file: "docs/02-module-1-ros2/ros2-architecture.mdx" - id: "module_1_ros2_code" agent: "lab_instructor" description: "Hands-on coding for ROS 2." prompt: | Write a tutorial: 'Your First ROS 2 Node'. - Create a Python (rclpy) Publisher node that simulates a 'Joint State' sensor. - Create a Subscriber node that reads this data. - Provide the full code in ```python blocks. - Explain the 'package.xml' and 'setup.py' configuration. output_file: "docs/02-module-1-ros2/first-node-python.mdx" # ========================================== # MODULE 2: DIGITAL TWIN (SIMULATION) # ========================================== - id: "module_2_urdf" agent: "content_writer" description: "Robot description formats." prompt: | Write 'Understanding URDF & SDF'. - Explain how we define a robot's body (Links) and muscles (Joints) in XML. - Explain the difference between URDF (standard) and SDF (Gazebo specific). - Show a code snippet of a simple URDF for a robotic leg. output_file: "docs/03-module-2-simulation/urdf-guide.mdx" - id: "module_2_gazebo" agent: "lab_instructor" description: "Setting up the simulation environment." prompt: | Write 'Simulating Physics in Gazebo'. - Guide the user on spawning the URDF into Gazebo. - Explain how to add Simulated Sensors (Lidar/Camera). - Use :::warning to check for 'Real Time Factor' drops if the CPU is weak. output_file: "docs/03-module-2-simulation/gazebo-physics.mdx" # ========================================== # MODULE 3: THE AI BRAIN (NVIDIA ISAAC) # ========================================== - id: "module_3_isaac_sim" agent: "content_writer" description: "Introduction to NVIDIA Omniverse and Isaac Sim." prompt: | Write 'Introduction to NVIDIA Isaac Sim'. - Explain the 'Omniverse' ecosystem and USD (Universal Scene Description) files. - Detail how Isaac Sim differs from Gazebo (Photorealism vs Pure Physics). - Explain 'Synthetic Data Generation' for training vision models. output_file: "docs/04-module-3-isaac/intro-isaac-sim.mdx" - id: "module_3_nav2" agent: "lab_instructor" description: "Navigation and SLAM." prompt: | Write 'Navigation & VSLAM with Isaac ROS'. - Explain how a robot maps a room using SLAM (Simultaneous Localization and Mapping). - Guide: Configuring 'Nav2' for a humanoid robot to walk from Point A to Point B avoiding obstacles. - Mention using the Intel RealSense D435i for depth data. output_file: "docs/04-module-3-isaac/nav2-vslam.mdx" # ========================================== # MODULE 4: VLA (VISION LANGUAGE ACTION) # ========================================== - id: "module_4_vla_theory" agent: "content_writer" description: "Theory of LLMs in Robotics." prompt: | Write 'Vision-Language-Action (VLA) Models'. - Explain how we connect LLMs (like GPT-4 or Gemini) to ROS 2 actions. - Concept: Translating "Clean the kitchen" -> [Navigate(Kitchen), Identify(Trash), Pick(Trash)]. - Introduce the 'Capstone Project: The Autonomous Humanoid'. output_file: "docs/05-module-4-vla/vla-theory.mdx" - id: "module_4_whisper" agent: "lab_instructor" description: "Voice control implementation." prompt: | Write 'Voice Control with OpenAI Whisper'. - Tutorial: Integrate OpenAI Whisper API to capture voice commands. - Code: A Python script that takes audio input and publishes a String to a ROS 2 topic `/voice_command`. - Add a <TranslationButton /> component placeholder for Urdu translation here. output_file: "docs/05-module-4-vla/voice-control.mdx"" --json --number 5 --short-name "user-auth" "Add user authentication"
      - PowerShell example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "name: "Physical_AI_Textbook_Generation" description: "Generates the complete Physical AI & Humanoid Robotics textbook." version: "1.0" # Global context provided to all tasks context: constitution: "sp.constitution" project_name: "Physical AI & Humanoid Robotics" author: "Mehma Rashid" tasks: # ========================================== # INTRODUCTION & ONBOARDING # ========================================== - id: "write_intro" agent: "content_writer" description: "Write the course landing page and introduction." prompt: | Write the 'Introduction to Physical AI'. - Define 'Embodied Intelligence'. - Explain why we are moving from Digital Agents (Chatbots) to Physical Agents (Robots). - Include a section on the 'Startup Opportunity' in robotics. - Add a <ChatBotContext topic="introduction" /> component at the bottom. output_file: "docs/intro.mdx" - id: "write_hardware_setup" agent: "content_writer" description: "Detailed hardware guide with safety warnings." prompt: | Write the 'Hardware & Lab Setup' guide. - Compare 'Digital Twin Workstation' (RTX 4070 Ti+) vs 'Edge Kit' (Jetson Orin Nano). - Detail the 'Robot Lab' options: Proxy (Unitree Go2) vs Humanoid (Unitree G1). - Use :::danger to warn about LiPo battery handling and VRAM limitations. - Use :::tip for suggesting Cloud Workstations (AWS g5.2xlarge) if local GPU is unavailable. output_file: "docs/01-setup/hardware-requirements.mdx" # ========================================== # MODULE 1: THE ROBOTIC NERVOUS SYSTEM (ROS 2) # ========================================== - id: "module_1_ros2_concepts" agent: "content_writer" description: "Core concepts of ROS 2." prompt: | Write 'Module 1: ROS 2 Architecture'. - Explain Nodes, Topics, Services, and Actions using a 'Nervous System' analogy. - Explain why we use ROS 2 Humble/Iron and Linux (Ubuntu 22.04). - Include a <PersonalizedContent> block that explains ROS differently for Python Developers vs C++ Developers. output_file: "docs/02-module-1-ros2/ros2-architecture.mdx" - id: "module_1_ros2_code" agent: "lab_instructor" description: "Hands-on coding for ROS 2." prompt: | Write a tutorial: 'Your First ROS 2 Node'. - Create a Python (rclpy) Publisher node that simulates a 'Joint State' sensor. - Create a Subscriber node that reads this data. - Provide the full code in ```python blocks. - Explain the 'package.xml' and 'setup.py' configuration. output_file: "docs/02-module-1-ros2/first-node-python.mdx" # ========================================== # MODULE 2: DIGITAL TWIN (SIMULATION) # ========================================== - id: "module_2_urdf" agent: "content_writer" description: "Robot description formats." prompt: | Write 'Understanding URDF & SDF'. - Explain how we define a robot's body (Links) and muscles (Joints) in XML. - Explain the difference between URDF (standard) and SDF (Gazebo specific). - Show a code snippet of a simple URDF for a robotic leg. output_file: "docs/03-module-2-simulation/urdf-guide.mdx" - id: "module_2_gazebo" agent: "lab_instructor" description: "Setting up the simulation environment." prompt: | Write 'Simulating Physics in Gazebo'. - Guide the user on spawning the URDF into Gazebo. - Explain how to add Simulated Sensors (Lidar/Camera). - Use :::warning to check for 'Real Time Factor' drops if the CPU is weak. output_file: "docs/03-module-2-simulation/gazebo-physics.mdx" # ========================================== # MODULE 3: THE AI BRAIN (NVIDIA ISAAC) # ========================================== - id: "module_3_isaac_sim" agent: "content_writer" description: "Introduction to NVIDIA Omniverse and Isaac Sim." prompt: | Write 'Introduction to NVIDIA Isaac Sim'. - Explain the 'Omniverse' ecosystem and USD (Universal Scene Description) files. - Detail how Isaac Sim differs from Gazebo (Photorealism vs Pure Physics). - Explain 'Synthetic Data Generation' for training vision models. output_file: "docs/04-module-3-isaac/intro-isaac-sim.mdx" - id: "module_3_nav2" agent: "lab_instructor" description: "Navigation and SLAM." prompt: | Write 'Navigation & VSLAM with Isaac ROS'. - Explain how a robot maps a room using SLAM (Simultaneous Localization and Mapping). - Guide: Configuring 'Nav2' for a humanoid robot to walk from Point A to Point B avoiding obstacles. - Mention using the Intel RealSense D435i for depth data. output_file: "docs/04-module-3-isaac/nav2-vslam.mdx" # ========================================== # MODULE 4: VLA (VISION LANGUAGE ACTION) # ========================================== - id: "module_4_vla_theory" agent: "content_writer" description: "Theory of LLMs in Robotics." prompt: | Write 'Vision-Language-Action (VLA) Models'. - Explain how we connect LLMs (like GPT-4 or Gemini) to ROS 2 actions. - Concept: Translating "Clean the kitchen" -> [Navigate(Kitchen), Identify(Trash), Pick(Trash)]. - Introduce the 'Capstone Project: The Autonomous Humanoid'. output_file: "docs/05-module-4-vla/vla-theory.mdx" - id: "module_4_whisper" agent: "lab_instructor" description: "Voice control implementation." prompt: | Write 'Voice Control with OpenAI Whisper'. - Tutorial: Integrate OpenAI Whisper API to capture voice commands. - Code: A Python script that takes audio input and publishes a String to a ROS 2 topic `/voice_command`. - Add a <TranslationButton /> component placeholder for Urdu translation here. output_file: "docs/05-module-4-vla/voice-control.mdx"" -Json -Number 5 -ShortName "user-auth" "Add user authentication"
      - PowerShell example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "name: "Physical_AI_Textbook_Generation" description: "Generates the complete Physical AI & Humanoid Robotics textbook." version: "1.0" # Global context provided to all tasks context: constitution: "sp.constitution" project_name: "Physical AI & Humanoid Robotics" author: "Mehma Rashid" tasks: # ========================================== # INTRODUCTION & ONBOARDING # ========================================== - id: "write_intro" agent: "content_writer" description: "Write the course landing page and introduction." prompt: | Write the 'Introduction to Physical AI'. - Define 'Embodied Intelligence'. - Explain why we are moving from Digital Agents (Chatbots) to Physical Agents (Robots). - Include a section on the 'Startup Opportunity' in robotics. - Add a <ChatBotContext topic="introduction" /> component at the bottom. output_file: "docs/intro.mdx" - id: "write_hardware_setup" agent: "content_writer" description: "Detailed hardware guide with safety warnings." prompt: | Write the 'Hardware & Lab Setup' guide. - Compare 'Digital Twin Workstation' (RTX 4070 Ti+) vs 'Edge Kit' (Jetson Orin Nano). - Detail the 'Robot Lab' options: Proxy (Unitree Go2) vs Humanoid (Unitree G1). - Use :::danger to warn about LiPo battery handling and VRAM limitations. - Use :::tip for suggesting Cloud Workstations (AWS g5.2xlarge) if local GPU is unavailable. output_file: "docs/01-setup/hardware-requirements.mdx" # ========================================== # MODULE 1: THE ROBOTIC NERVOUS SYSTEM (ROS 2) # ========================================== - id: "module_1_ros2_concepts" agent: "content_writer" description: "Core concepts of ROS 2." prompt: | Write 'Module 1: ROS 2 Architecture'. - Explain Nodes, Topics, Services, and Actions using a 'Nervous System' analogy. - Explain why we use ROS 2 Humble/Iron and Linux (Ubuntu 22.04). - Include a <PersonalizedContent> block that explains ROS differently for Python Developers vs C++ Developers. output_file: "docs/02-module-1-ros2/ros2-architecture.mdx" - id: "module_1_ros2_code" agent: "lab_instructor" description: "Hands-on coding for ROS 2." prompt: | Write a tutorial: 'Your First ROS 2 Node'. - Create a Python (rclpy) Publisher node that simulates a 'Joint State' sensor. - Create a Subscriber node that reads this data. - Provide the full code in ```python blocks. - Explain the 'package.xml' and 'setup.py' configuration. output_file: "docs/02-module-1-ros2/first-node-python.mdx" # ========================================== # MODULE 2: DIGITAL TWIN (SIMULATION) # ========================================== - id: "module_2_urdf" agent: "content_writer" description: "Robot description formats." prompt: | Write 'Understanding URDF & SDF'. - Explain how we define a robot's body (Links) and muscles (Joints) in XML. - Explain the difference between URDF (standard) and SDF (Gazebo specific). - Show a code snippet of a simple URDF for a robotic leg. output_file: "docs/03-module-2-simulation/urdf-guide.mdx" - id: "module_2_gazebo" agent: "lab_instructor" description: "Setting up the simulation environment." prompt: | Write 'Simulating Physics in Gazebo'. - Guide the user on spawning the URDF into Gazebo. - Explain how to add Simulated Sensors (Lidar/Camera). - Use :::warning to check for 'Real Time Factor' drops if the CPU is weak. output_file: "docs/03-module-2-simulation/gazebo-physics.mdx" # ========================================== # MODULE 3: THE AI BRAIN (NVIDIA ISAAC) # ========================================== - id: "module_3_isaac_sim" agent: "content_writer" description: "Introduction to NVIDIA Omniverse and Isaac Sim." prompt: | Write 'Introduction to NVIDIA Isaac Sim'. - Explain the 'Omniverse' ecosystem and USD (Universal Scene Description) files. - Detail how Isaac Sim differs from Gazebo (Photorealism vs Pure Physics). - Explain 'Synthetic Data Generation' for training vision models. output_file: "docs/04-module-3-isaac/intro-isaac-sim.mdx" - id: "module_3_nav2" agent: "lab_instructor" description: "Navigation and SLAM." prompt: | Write 'Navigation & VSLAM with Isaac ROS'. - Explain how a robot maps a room using SLAM (Simultaneous Localization and Mapping). - Guide: Configuring 'Nav2' for a humanoid robot to walk from Point A to Point B avoiding obstacles. - Mention using the Intel RealSense D435i for depth data. output_file: "docs/04-module-3-isaac/nav2-vslam.mdx" # ========================================== # MODULE 4: VLA (VISION LANGUAGE ACTION) # ========================================== - id: "module_4_vla_theory" agent: "content_writer" description: "Theory of LLMs in Robotics." prompt: | Write 'Vision-Language-Action (VLA) Models'. - Explain how we connect LLMs (like GPT-4 or Gemini) to ROS 2 actions. - Concept: Translating "Clean the kitchen" -> [Navigate(Kitchen), Identify(Trash), Pick(Trash)]. - Introduce the 'Capstone Project: The Autonomous Humanoid'. output_file: "docs/05-module-4-vla/vla-theory.mdx" - id: "module_4_whisper" agent: "lab_instructor" description: "Voice control implementation." prompt: | Write 'Voice Control with OpenAI Whisper'. - Tutorial: Integrate OpenAI Whisper API to capture voice commands. - Code: A Python script that takes audio input and publishes a String to a ROS 2 topic `/voice_command`. - Add a <TranslationButton /> component placeholder for Urdu translation here. output_file: "docs/05-module-4-vla/voice-control.mdx"" -Json -Number 5 -ShortName "user-auth" "Add user authentication"
   
   **IMPORTANT**:
   - Check all three sources (remote branches, local branches, specs directories) to find the highest number
   - Only match branches/directories with the exact short-name pattern
   - If no existing branches/directories found with this short-name, start with number 1
   - You must only ever run this script once per feature
   - The JSON is provided in the terminal as output - always refer to it to get the actual content you're looking for
   - The JSON output will contain BRANCH_NAME and SPEC_FILE paths
   - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot")

3. Load `.specify/templates/spec-template.md` to understand required sections.

4. Follow this execution flow:

    1. Parse user description from Input
       If empty: ERROR "No feature description provided"
    2. Extract key concepts from description
       Identify: actors, actions, data, constraints
    3. For unclear aspects:
       - Make informed guesses based on context and industry standards
       - Only mark with [NEEDS CLARIFICATION: specific question] if:
         - The choice significantly impacts feature scope or user experience
         - Multiple reasonable interpretations exist with different implications
         - No reasonable default exists
       - **LIMIT: Maximum 3 [NEEDS CLARIFICATION] markers total**
       - Prioritize clarifications by impact: scope > security/privacy > user experience > technical details
    4. Fill User Scenarios & Testing section
       If no clear user flow: ERROR "Cannot determine user scenarios"
    5. Generate Functional Requirements
       Each requirement must be testable
       Use reasonable defaults for unspecified details (document assumptions in Assumptions section)
    6. Define Success Criteria
       Create measurable, technology-agnostic outcomes
       Include both quantitative metrics (time, performance, volume) and qualitative measures (user satisfaction, task completion)
       Each criterion must be verifiable without implementation details
    7. Identify Key Entities (if data involved)
    8. Return: SUCCESS (spec ready for planning)

5. Write the specification to SPEC_FILE using the template structure, replacing placeholders with concrete details derived from the feature description (arguments) while preserving section order and headings.

6. **Specification Quality Validation**: After writing the initial spec, validate it against quality criteria:

   a. **Create Spec Quality Checklist**: Generate a checklist file at `FEATURE_DIR/checklists/requirements.md` using the checklist template structure with these validation items:

      ```markdown
      # Specification Quality Checklist: [FEATURE NAME]
      
      **Purpose**: Validate specification completeness and quality before proceeding to planning
      **Created**: [DATE]
      **Feature**: [Link to spec.md]
      
      ## Content Quality
      
      - [ ] No implementation details (languages, frameworks, APIs)
      - [ ] Focused on user value and business needs
      - [ ] Written for non-technical stakeholders
      - [ ] All mandatory sections completed
      
      ## Requirement Completeness
      
      - [ ] No [NEEDS CLARIFICATION] markers remain
      - [ ] Requirements are testable and unambiguous
      - [ ] Success criteria are measurable
      - [ ] Success criteria are technology-agnostic (no implementation details)
      - [ ] All acceptance scenarios are defined
      - [ ] Edge cases are identified
      - [ ] Scope is clearly bounded
      - [ ] Dependencies and assumptions identified
      
      ## Feature Readiness
      
      - [ ] All functional requirements have clear acceptance criteria
      - [ ] User scenarios cover primary flows
      - [ ] Feature meets measurable outcomes defined in Success Criteria
      - [ ] No implementation details leak into specification
      
      ## Notes
      
      - Items marked incomplete require spec updates before `/sp.clarify` or `/sp.plan`
      ```

   b. **Run Validation Check**: Review the spec against each checklist item:
      - For each item, determine if it passes or fails
      - Document specific issues found (quote relevant spec sections)

   c. **Handle Validation Results**:

      - **If all items pass**: Mark checklist complete and proceed to step 6

      - **If items fail (excluding [NEEDS CLARIFICATION])**:
        1. List the failing items and specific issues
        2. Update the spec to address each issue
        3. Re-run validation until all items pass (max 3 iterations)
        4. If still failing after 3 iterations, document remaining issues in checklist notes and warn user

      - **If [NEEDS CLARIFICATION] markers remain**:
        1. Extract all [NEEDS CLARIFICATION: ...] markers from the spec
        2. **LIMIT CHECK**: If more than 3 markers exist, keep only the 3 most critical (by scope/security/UX impact) and make informed guesses for the rest
        3. For each clarification needed (max 3), present options to user in this format:

           ```markdown
           ## Question [N]: [Topic]
           
           **Context**: [Quote relevant spec section]
           
           **What we need to know**: [Specific question from NEEDS CLARIFICATION marker]
           
           **Suggested Answers**:
           
           | Option | Answer | Implications |
           |--------|--------|--------------|
           | A      | [First suggested answer] | [What this means for the feature] |
           | B      | [Second suggested answer] | [What this means for the feature] |
           | C      | [Third suggested answer] | [What this means for the feature] |
           | Custom | Provide your own answer | [Explain how to provide custom input] |
           
           **Your choice**: _[Wait for user response]_
           ```

        4. **CRITICAL - Table Formatting**: Ensure markdown tables are properly formatted:
           - Use consistent spacing with pipes aligned
           - Each cell should have spaces around content: `| Content |` not `|Content|`
           - Header separator must have at least 3 dashes: `|--------|`
           - Test that the table renders correctly in markdown preview
        5. Number questions sequentially (Q1, Q2, Q3 - max 3 total)
        6. Present all questions together before waiting for responses
        7. Wait for user to respond with their choices for all questions (e.g., "Q1: A, Q2: Custom - [details], Q3: B")
        8. Update the spec by replacing each [NEEDS CLARIFICATION] marker with the user's selected or provided answer
        9. Re-run validation after all clarifications are resolved

   d. **Update Checklist**: After each validation iteration, update the checklist file with current pass/fail status

7. Report completion with branch name, spec file path, checklist results, and readiness for the next phase (`/sp.clarify` or `/sp.plan`).

**NOTE**: The script creates and checks out the new branch and initializes the spec file before writing.

## General Guidelines

## Quick Guidelines

- Focus on **WHAT** users need and **WHY**.
- Avoid HOW to implement (no tech stack, APIs, code structure).
- Written for business stakeholders, not developers.
- DO NOT create any checklists that are embedded in the spec. That will be a separate command.

### Section Requirements

- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation

When creating this spec from a user prompt:

1. **Make informed guesses**: Use context, industry standards, and common patterns to fill gaps
2. **Document assumptions**: Record reasonable defaults in the Assumptions section
3. **Limit clarifications**: Maximum 3 [NEEDS CLARIFICATION] markers - use only for critical decisions that:
   - Significantly impact feature scope or user experience
   - Have multiple reasonable interpretations with different implications
   - Lack any reasonable default
4. **Prioritize clarifications**: scope > security/privacy > user experience > technical details
5. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
6. **Common areas needing clarification** (only if no reasonable default exists):
   - Feature scope and boundaries (include/exclude specific use cases)
   - User types and permissions (if multiple conflicting interpretations possible)
   - Security/compliance requirements (when legally/financially significant)

**Examples of reasonable defaults** (don't ask about these):

- Data retention: Industry-standard practices for the domain
- Performance targets: Standard web/mobile app expectations unless specified
- Error handling: User-friendly messages with appropriate fallbacks
- Authentication method: Standard session-based or OAuth2 for web apps
- Integration patterns: RESTful APIs unless specified otherwise

### Success Criteria Guidelines

Success criteria must be:

1. **Measurable**: Include specific metrics (time, percentage, count, rate)
2. **Technology-agnostic**: No mention of frameworks, languages, databases, or tools
3. **User-focused**: Describe outcomes from user/business perspective, not system internals
4. **Verifiable**: Can be tested/validated without knowing implementation details

**Good examples**:

- "Users can complete checkout in under 3 minutes"
- "System supports 10,000 concurrent users"
- "95% of searches return results in under 1 second"
- "Task completion rate improves by 40%"

**Bad examples** (implementation-focused):

- "API response time is under 200ms" (too technical, use "Users see results instantly")
- "Database can handle 1000 TPS" (implementation detail, use user-facing metric)
- "React components render efficiently" (framework-specific)
- "Redis cache hit rate above 80%" (technology-specific)

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

I have created the feature specification for 'Physical AI Textbook Generation'.
- **Branch:** `001-textbook-generation`
- **Specification File:** `specs/001-textbook-generation/spec.md`
- **Checklist:** `specs/001-textbook-generation/checklists/requirements.md` (All checks passed)

The feature is now ready for the next phase, which would be `/sp.plan`.

## Outcome

- ✅ Impact: A new feature specification has been created, defining the user stories, requirements, and success criteria for generating the textbook content.
- 🧪 Tests: No tests were run.
- 📁 Files: 
  - `specs/001-textbook-generation/spec.md`
  - `specs/001-textbook-generation/checklists/requirements.md`
- 🔁 Next prompts: The user can now proceed with the planning phase for this feature using `/sp.plan`.
- 🧠 Reflection: The manual creation of the feature branch and spec file was a safe alternative to using the PowerShell script with a very long and complex argument. The process of deriving user stories and requirements from a structured task list worked well.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
