# Implementation Plan: Website Deployment and Embedding Generation

**Branch**: `003-deploy-website-embeddings` | **Date**: 2025-12-19 | **Spec**: [specs/003-deploy-website-embeddings/spec.md](../spec.md)
**Input**: Feature specification from `specs/003-deploy-website-embeddings/spec.md`

## Summary

This feature involves deploying the existing Docusaurus documentation site to Vercel (at `https://ai-native-book-psi.vercel.app/`) and implementing a Python-based backend pipeline to ingest the documentation content into a vector database (Qdrant) using Cohere embeddings. This setup enables a RAG (Retrieval-Augmented Generation) chatbot.

## Technical Context

**Language/Version**: Python 3.10+ (Backend), Node.js (Frontend)
**Primary Dependencies**: `cohere`, `qdrant-client`, `beautifulsoup4`, `langchain-text-splitters`, `lxml` (for sitemap parsing)
**Storage**: Qdrant Cloud (Vector Database)
**Testing**: `pytest` (Backend)
**Target Platform**: Vercel (`https://ai-native-book-psi.vercel.app/`), Local/Server Script (Backend)
**Project Type**: Web Application + Data Pipeline
**Performance Goals**: Embeddings retrieval < 200ms
**Constraints**: Free Tier limits of Cohere and Qdrant
**Scale/Scope**: < 100 documentation pages, < 10k vectors

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Scope**: Aligned with Module 4 (Vision-Language-Action) and the overall goal of building an intelligent agent.
- **Dependencies**: Uses standard, specified tools (Cohere, Qdrant).
- **Code Standards**: Adheres to Python best practices (using `uv`).

## Project Structure

### Documentation (this feature)

```text
specs/003-deploy-website-embeddings/
├── plan.md              # This file
├── research.md          # Technical decisions and architecture
├── data-model.md        # Schema for chunks and vectors
├── quickstart.md        # Setup guide for the backend
└── tasks.md             # (To be created)
```

### Source Code

```text
AI-BOOK/                # Frontend (Docusaurus)
├── src/
├── docs/               # Source of truth for content
└── ...

backend/                # New Backend Directory
├── .venv/              # Virtual environment (managed by uv)
├── main.py             # RAG pipeline script
├── .env                # Environment variables (API keys)
└── pyproject.toml      # Dependency management
```

**Structure Decision**: A separate `backend/` folder keeps the Python logic isolated from the Docusaurus React frontend. The `main.py` will implement the `get_all_urls` (using `sitemap.xml`), `extract_text_from_url`, `chunk_text`, `embed`, `create_collection`, and `save_chunk_to_qdrant` functions as requested.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | | |