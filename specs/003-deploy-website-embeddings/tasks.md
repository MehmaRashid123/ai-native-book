---
description: "Task list for Website Deployment and Embedding Generation"
---

# Tasks: Website Deployment and Embedding Generation

**Input**: Design documents from `specs/003-deploy-website-embeddings/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Tests are OPTIONAL and not explicitly requested in the spec, but basic verification scripts are included as part of the stories.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: [US1], [US2], [US3] (maps to user stories from spec.md)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend directory structure `backend/` in repository root
- [x] T002 Initialize `uv` project in `backend/` (run `uv init`)
- [x] T003 Install Python dependencies (`cohere`, `qdrant-client`, `beautifulsoup4`, `langchain-text-splitters`, `lxml`, `python-dotenv`) in `backend/`
- [x] T004 Create `.env` file template in `backend/.env` with placeholders for `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`
- [x] T005 [P] Create `backend/main.py` file with initial imports and empty main execution block

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 [US2] Implement environment variable loading in `backend/main.py` using `dotenv`
- [x] T007 [US2] Initialize Cohere and Qdrant clients in `backend/main.py` (global or main scope)
- [x] T008 [US2] Define data classes/structures for `DocumentChunk` in `backend/main.py` matching `data-model.md`

**Checkpoint**: Foundation ready - clients are initialized and env vars are loadable.

---

## Phase 3: User Story 1 - Deploy Book Website (Priority: P1) 🎯 MVP

**Goal**: Ensure the Docusaurus site is deployable and accessible at the target URL.

**Independent Test**: Visit `https://ai-native-book-psi.vercel.app/` and confirm content loads.

### Implementation for User Story 1

- [x] T009 [US1] Verify Docusaurus build locally by running `npm run build` in `AI-BOOK/`
- [x] T010 [US1] Create a `vercel.json` configuration file in `AI-BOOK/` (if needed for specific overrides, else optional)
- [x] T011 [US1] Document manual deployment steps or verify existing deployment to `https://ai-native-book-psi.vercel.app/` in `specs/003-deploy-website-embeddings/deployment_log.md` (Self-verification task)

**Checkpoint**: Website is live and accessible.

---

## Phase 4: User Story 2 - Ingest and Embed Content (Priority: P1)

**Goal**: Scrape content from the live site, generate embeddings, and store them in Qdrant.

**Independent Test**: Run `main.py` and verify vector count in Qdrant Cloud dashboard.

### Implementation for User Story 2

- [x] T012 [US2] Implement `get_all_urls(sitemap_url)` function in `backend/main.py` to parse `sitemap.xml`
- [x] T013 [US2] Implement `extract_text_from_url(url)` function in `backend/main.py` using `BeautifulSoup`
- [x] T014 [US2] Implement `chunk_text(text)` function in `backend/main.py` using `RecursiveCharacterTextSplitter`
- [x] T015 [US2] Implement `embed(text_chunks)` function in `backend/main.py` using Cohere client
- [x] T016 [US2] Implement `create_collection(collection_name)` function in `backend/main.py` to ensure Qdrant collection exists
- [x] T017 [US2] Implement `save_chunk_to_qdrant(chunks, embeddings)` function in `backend/main.py` to upsert vectors
- [x] T018 [US2] Orchestrate the pipeline in the `if __name__ == "__main__":` block of `backend/main.py`

**Checkpoint**: Data is indexed in Qdrant.

---

## Phase 5: User Story 3 - Validate Retrieval (Priority: P2)

**Goal**: Verify that the RAG pipeline actually retrieves relevant context.

**Independent Test**: Run a specific test query function and print results.

### Implementation for User Story 3

- [x] T019 [US3] Add `search(query)` function to `backend/main.py` to perform similarity search in Qdrant
- [x] T020 [US3] Create a separate verification script `backend/verify_rag.py` (or add to main) that runs a sample query (e.g., "What is ROS 2?") and asserts the presence of expected keywords in the result.

**Checkpoint**: Retrieval system is verified functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and final documentation.

- [x] T021 [P] Update `specs/003-deploy-website-embeddings/quickstart.md` with any new commands or findings
- [x] T022 Add basic logging to `backend/main.py` for pipeline progress tracking
- [x] T023 Add error handling for network requests (crawling and API calls) in `backend/main.py`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies.
- **Foundational (Phase 2)**: Depends on Setup.
- **User Story 1 (Deploy)**: Independent of Backend tasks, depends on Frontend source.
- **User Story 2 (Ingest)**: Depends on Foundational completion AND User Story 1 (needs live URL/sitemap).
- **User Story 3 (Validate)**: Depends on User Story 2 completion (needs data in Qdrant).

### Parallel Opportunities

- T009 (Frontend Build) can run in parallel with T001-T008 (Backend Setup).
- T012, T013, T014 (Text Processing) can be implemented effectively in parallel if functions are defined clearly.
