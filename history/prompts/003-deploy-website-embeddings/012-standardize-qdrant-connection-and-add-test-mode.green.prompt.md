---
id: 012
title: standardize-qdrant-connection-and-add-test-mode
stage: green
date: 2025-12-19
surface: agent
model: gemini
feature: deploy-website-embeddings
branch: 003-deploy-website-embeddings
user: user
command: python main.py test
labels: [bugfix, qdrant, cloud, diagnostic]
links:
  spec: specs/003-deploy-website-embeddings/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/main.py
tests: []
---

## Prompt

The user still faced Qdrant 404s. The previous attempt to force host/port logic didn't resolve it.

## Response snapshot

1.  **Simplified Connection**: Reverted to the standard `url` parameter in `QdrantClient`, which is the officially recommended way for Cloud.
2.  **Added Test Mode**: Added `python main.py test` to verify connections to Cohere and Qdrant before running heavy tasks.
3.  **URL Sanitization**: The script now automatically handles the presence/absence of `https://` and strips port `:6333` if provided in the environment variable.

## Outcome

- ✅ Impact: Clearer error reporting and standardized connection logic.
- 🧪 Tests: `test_connections()` function added.
- 📁 Files: `backend/main.py`
- 🧠 Reflection: When client-side forced logic fails, reverting to the most basic, documentation-aligned configuration is the safest path to resolution.
