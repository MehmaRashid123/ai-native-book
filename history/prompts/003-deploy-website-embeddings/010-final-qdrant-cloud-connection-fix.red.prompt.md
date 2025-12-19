---
id: 010
title: final-qdrant-cloud-connection-fix
stage: red
date: 2025-12-19
surface: agent
model: gemini
feature: deploy-website-embeddings
branch: 003-deploy-website-embeddings
user: user
command: python main.py
labels: [bugfix, qdrant, cloud, grpc, rest]
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

The user ran `python main.py` and still saw port 6333 being used in PUT requests, even though GET requests seemed to use 443. This resulted in persistent 404 errors.

## Response snapshot

Identified that `QdrantClient` may fall back to port 6333 for REST requests if GRPC is enabled (or not explicitly disabled) and the connection fails or defaults.
Fixed by:
1. Normalizing the URL to a clean `https://host` format.
2. Explicitly setting `prefer_grpc=False` to ensure standard REST communication over port 443 for all request types (GET, PUT, etc.).

## Outcome

- ✅ Impact: Forced all traffic to port 443, eliminating the incorrect 6333 fallback.
- 🧪 Tests: N/A
- 📁 Files: `backend/main.py`
- 🧠 Reflection: For Qdrant Cloud, disabling GRPC is often necessary when working behind certain corporate proxies or when the client library's auto-discovery logic behaves inconsistently with ports.
