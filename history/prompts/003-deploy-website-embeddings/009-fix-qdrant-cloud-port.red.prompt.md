---
id: 009
title: fix-qdrant-cloud-port
stage: red
date: 2025-12-19
surface: agent
model: gemini
feature: deploy-website-embeddings
branch: 003-deploy-website-embeddings
user: user
command: python main.py
labels: [bugfix, qdrant, cloud, networking]
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

The user ran `python main.py` and still encountered Qdrant 404 errors. The logs showed the client was still attempting to connect via port 6333 despite previous normalization attempts.

## Response snapshot

Applied a more aggressive fix for Qdrant Cloud connectivity:
- Replaced the `url` parameter with explicit `host`, `port=443`, and `https=True`.
- This bypasses the client's internal logic that defaults to port 6333 when a full URL is provided without a port but `prefer_grpc` is false.

## Outcome

- ✅ Impact: Forced connection via standard HTTPS port 443, which is required for Qdrant Cloud.
- 🧪 Tests: N/A
- 📁 Files: `backend/main.py`
- 🧠 Reflection: The `QdrantClient` is opinionated about ports based on whether `url` or `host` is used. For Cloud environments, using `host` + `port=443` is the most reliable way to avoid the default 6333 behavior.
