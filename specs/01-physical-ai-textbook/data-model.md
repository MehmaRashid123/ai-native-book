# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: 01-physical-ai-textbook
**Date**: 2025-12-10

## Entity Definitions

### User
**Description**: A registered individual accessing the textbook content

**Fields**:
- `user_id` (string, required): Unique identifier for the user
- `email` (string, required): User's email address for authentication
- `gpu_model` (string, optional): User's GPU model for hardware compatibility checks
- `programming_level` (string, optional): User's programming experience level (beginner, intermediate, advanced)
- `preferred_language` (string, optional): Default language preference (default: "en")
- `authentication_status` (enum, required): ["authenticated", "guest"]
- `created_at` (datetime, required): Account creation timestamp
- `updated_at` (datetime, required): Last update timestamp

**Validation Rules**:
- Email must be valid email format
- GPU model must be from predefined list or "unknown"
- Programming level must be one of: "beginner", "intermediate", "advanced"

### Chapter
**Description**: A section of educational content within the textbook

**Fields**:
- `chapter_id` (string, required): Unique identifier for the chapter
- `module_number` (integer, required): Module number (1-4)
- `title` (string, required): Chapter title
- `content` (string, required): Chapter content in markdown format
- `language_versions` (object, required): Map of language codes to content versions
- `associated_skills` (array, optional): List of ROS/robotics skills covered
- `prerequisite_chapters` (array, optional): List of chapter IDs that must be completed first
- `created_at` (datetime, required): Creation timestamp
- `updated_at` (datetime, required): Last update timestamp

**Validation Rules**:
- Module number must be between 1 and 4
- Content must be valid markdown
- Language versions must include at least English ("en")

### Module
**Description**: A collection of chapters forming a complete learning unit

**Fields**:
- `module_id` (string, required): Unique identifier for the module
- `module_number` (integer, required): Module number (1-4)
- `title` (string, required): Module title
- `description` (string, required): Brief description of the module
- `prerequisite_modules` (array, optional): List of module IDs that must be completed first
- `chapters` (array, required): List of chapter IDs in this module
- `estimated_completion_hours` (number, optional): Estimated time to complete module
- `learning_objectives` (array, required): List of learning objectives for the module
- `created_at` (datetime, required): Creation timestamp
- `updated_at` (datetime, required): Last update timestamp

**Validation Rules**:
- Module number must be between 1 and 4
- Must have at least one chapter
- Learning objectives must be non-empty array

### AgentQuery
**Description**: A user's question to the Vector agent

**Fields**:
- `query_id` (string, required): Unique identifier for the query
- `user_id` (string, optional): User ID if authenticated, null for guest
- `query_text` (string, required): The text of the user's question
- `context_chapter` (string, optional): Current chapter ID for context awareness
- `response` (string, required): The agent's response to the query
- `timestamp` (datetime, required): When the query was made
- `query_type` (enum, required): ["content_question", "hardware_check", "code_generation", "other"]
- `is_hallucination_checked` (boolean, required): Whether response was verified against textbook content

**Validation Rules**:
- Query text must be non-empty
- Response must be non-empty
- Query type must be one of the defined values

### ContentEmbedding
**Description**: Vector embeddings for textbook content used in RAG system

**Fields**:
- `embedding_id` (string, required): Unique identifier for the embedding
- `content_id` (string, required): ID of the original content (chapter/section)
- `content_type` (enum, required): ["chapter", "section", "code_block", "concept"]
- `content_text` (string, required): Original text content
- `vector` (array, required): The embedding vector (array of floats)
- `language` (string, required): Language code (e.g., "en", "ur")
- `metadata` (object, optional): Additional metadata for the embedding
- `created_at` (datetime, required): Creation timestamp

**Validation Rules**:
- Vector must be of consistent dimension (e.g., 1536 for OpenAI embeddings)
- Content text must be non-empty
- Language must be supported by the system

## Relationships

### User → AgentQuery
- One-to-many: A user can make multiple queries
- Foreign key: `AgentQuery.user_id` references `User.user_id`

### Module → Chapter
- One-to-many: A module contains multiple chapters
- Foreign key: `Chapter.module_number` relates to `Module.module_number`

### Chapter → AgentQuery
- One-to-many: A chapter can be referenced by multiple queries for context
- Field: `AgentQuery.context_chapter` references `Chapter.chapter_id`

## State Transitions

### User Authentication State
```
Guest → Authenticated (login/signup)
Authenticated → Guest (logout)
```

### Content Approval State
```
Draft → Review → Published
Published → Review (update)
```

## Indexes

### Required Indexes
- `User.email` (unique)
- `Chapter.chapter_id` (unique)
- `Module.module_id` (unique)
- `AgentQuery.timestamp` (for chronological ordering)
- `ContentEmbedding.content_id` (for content lookup)