# Data Model: Physical AI Textbook Generation

This document outlines the key data entities for the textbook generation feature.

## Entities

### Textbook
- **Description**: The final collection of all generated documents, forming the complete textbook.
- **Attributes**:
    - `title`: The title of the textbook (e.g., "Physical AI & Humanoid Robotics").
    - `author`: The author of the textbook.
    - `version`: The version of the textbook.

### Module
- **Description**: A logical grouping of related content, corresponding to a section of the textbook.
- **Attributes**:
    - `name`: The name of the module (e.g., "The Robotic Nervous System (ROS 2)").
    - `focus`: The key topics covered in the module.

### Document
- **Description**: A single `.mdx` file representing a chapter or tutorial within a module.
- **Attributes**:
    - `id`: A unique identifier for the document generation task.
    - `output_file`: The path where the generated `.mdx` file will be saved.
    - `sidebar_label`: The label to be used in the Docusaurus sidebar.
    - `sidebar_position`: The position of the document in the sidebar.

### Agent
- **Description**: An AI entity responsible for generating content.
- **Attributes**:
    - `name`: The name of the agent (e.g., "content_writer").
    - `role`: The role of the agent (e.g., "Technical Author").
    - `model`: The underlying AI model used by the agent.
    - `skills`: A list of skills the agent possesses.

### Skill
- **Description**: A specific capability of an agent, defined by a prompt template.
- **Attributes**:
    - `name`: The name of the skill (e.g., "write_chapter").
    - `description`: A description of what the skill does.
    - `prompt_template`: The template used to generate content.
