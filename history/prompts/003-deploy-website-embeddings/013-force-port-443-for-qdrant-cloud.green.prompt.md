---
id: 013
title: force-port-443-for-qdrant-cloud
stage: green
date: 2025-12-19
surface: agent
model: gemini
feature: deploy-website-embeddings
branch: 003-deploy-website-embeddings
user: user
command: python main.py ingest
labels: [bugfix, qdrant, cloud, networking, port-fix]
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

The user still faced Qdrant 404s. The logs showed the client library was still defaulting to port 6333 despite passing an `https` URL.

## Response snapshot

Applied a definitive fix for Qdrant Cloud connectivity:
- The `QdrantClient` library automatically appends `:6333` if no port is specified in the `url` parameter.
- Qdrant Cloud clusters are only accessible on port `443` (standard HTTPS).
- Re-implemented the connection logic to explicitly append `:443` to the host, ensuring the library does not use its default port.

## Outcome

- ✅ Impact: Forced all traffic to the correct Qdrant Cloud API port.
- 🧪 Tests: N/A
- 📁 Files: `backend/main.py`
- 🧠 Reflection: Library defaults (like 6333 for Qdrant) often conflict with managed cloud services (like port 443 for Qdrant Cloud). Explicitly defining the port in the connection string is the most robust override.
