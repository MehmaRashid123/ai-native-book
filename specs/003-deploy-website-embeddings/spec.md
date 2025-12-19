# Feature Specification: Website Deployment and Embedding Generation

**Feature Branch**: `003-deploy-website-embeddings`  
**Created**: 2025-12-19  
**Status**: Draft  
**Input**: User description: "Website Deployment & Embeddings Target audience: Developers and technical team building the RAG chatbot for the book Focus: Deploy book website, generate embeddings from content, and store them in a vector database Success criteria: - Book website deployed and accessible via live URLs - Text content from the book processed and converted into embeddings - Embeddings successfully stored in Qdrant Cloud Free Tier - Embeddings generation uses Cohere models - Retrieval-ready vector database validated with test queries - Documentation provided for embedding and database setup"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Deploy Book Website (Priority: P1)

The developer deploys the Docusaurus-based book website to a public hosting provider to make the content accessible to readers.

**Why this priority**: The website is the primary product; without it, there is no content to read or index.

**Independent Test**: Can be tested by running the deployment command and verifying the site loads at the public URL.

**Acceptance Scenarios**:

1. **Given** a configured Docusaurus project, **When** the developer runs the deployment command, **Then** the site is built and pushed to the hosting provider.
2. **Given** the deployment completes, **When** a user visits the site URL, **Then** the home page and documentation chapters load correctly.

---

### User Story 2 - Ingest and Embed Content (Priority: P1)

The developer runs a script to extract text content from the book's source files, generate embeddings using Cohere, and store them in Qdrant.

**Why this priority**: This populates the knowledge base required for the RAG chatbot.

**Independent Test**: Can be tested by running the ingestion script and inspecting the Qdrant dashboard for new vectors.

**Acceptance Scenarios**:

1. **Given** valid Cohere and Qdrant API keys, **When** the ingestion script is executed, **Then** all MDX files in the `docs/` directory are processed.
2. **Given** the script completes, **When** the Qdrant collection is inspected, **Then** it contains vector entries with appropriate metadata (source URL, title, text chunk).

---

### User Story 3 - Validate Retrieval (Priority: P2)

The developer runs a test query against the vector database to ensure the embeddings are semantically relevant and retrieval works as expected.

**Why this priority**: Validates that the stored data is actually usable for RAG.

**Independent Test**: Can be tested by running a standalone query script and checking the relevance of the output.

**Acceptance Scenarios**:

1. **Given** a populated Qdrant collection, **When** a test query (e.g., "What is ROS 2?") is run, **Then** the system returns the "Introduction to ROS 2" section as a top result.

### Edge Cases

- **Missing/Invalid API Keys**: System should report a clear error message if Cohere or Qdrant API keys are missing or invalid, preventing the ingestion process from starting.
- **API Rate Limits**: System should handle rate limiting errors from the embedding provider (Cohere) by pausing or failing gracefully with a descriptive message.
- **Malformed Content**: If a documentation file cannot be parsed, the system should log a warning and skip the file rather than crashing the entire pipeline.
- **Network Failures**: If connection to the vector database fails during ingestion, the system should report the failure and not mark the process as successful.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a mechanism to deploy the Docusaurus site to a public URL (e.g., GitHub Pages).
- **FR-002**: System MUST provide a capability to traverse and parse MDX files from the `docs/` directory.
- **FR-003**: System MUST chunk text content appropriately for embedding.
- **FR-004**: System MUST use the specified embedding provider (Cohere) to generate vector embeddings for each text chunk.
- **FR-005**: System MUST store embeddings and metadata (content, title, URL) in the specified vector database (Qdrant Cloud).
- **FR-006**: System MUST allow configuration of API keys (Cohere, Qdrant) via environment variables.
- **FR-007**: System MUST provide a verification script that performs a similarity search query.
- **FR-008**: Documentation MUST be provided explaining how to set up keys and run the ingestion pipeline.

### Key Entities

- **Document Chunk**: Represents a segment of a book chapter. Attributes: `content`, `page_title`, `url_slug`, `chunk_id`.
- **Vector Record**: The storage entity. Attributes: `vector` (float array), `payload` (JSON of Document Chunk metadata).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Book website is accessible via a public HTTP/HTTPS URL with 200 OK status.
- **SC-002**: The vector database collection contains a vector count matching the number of generated text chunks.
- **SC-003**: Test queries return relevant document chunks (top-3 matches include the correct chapter) in under 2 seconds.
- **SC-004**: Ingestion process completes without errors for the entire current book content.