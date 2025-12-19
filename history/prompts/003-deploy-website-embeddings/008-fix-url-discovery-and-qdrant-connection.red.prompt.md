---
id: 008
title: fix-url-discovery-and-qdrant-connection
stage: red
date: 2025-12-19
surface: agent
model: gemini
feature: deploy-website-embeddings
branch: 003-deploy-website-embeddings
user: user
command: python main.py
labels: [bugfix, qdrant, docusaurus, sitemap]
links:
  spec: specs/003-deploy-website-embeddings/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/main.py
 - AI-BOOK/docusaurus.config.ts
tests: []
---

## Prompt

The user ran `python main.py` and encountered:
1. Qdrant 404 error when connecting to Cloud.
2. Sitemap URLs pointing to `your-docusaurus-site.example.com` instead of the live site.

## Response snapshot

Applied fixes for ingestion pipeline failures:
1.  **Docusaurus Config**: Updated `url` in `docusaurus.config.ts` to `https://ai-native-book-psi.vercel.app`.
2.  **Sitemap Robustness**: Added logic to `main.py` to replace the placeholder domain with the actual domain during discovery.
3.  **Qdrant Cloud Normalization**: Added logic to remove port `:6333` from Qdrant Cloud URLs, as they use port 443 (HTTPS) by default.

## Outcome

- ✅ Impact: Ingestion pipeline is now more resilient to configuration mismatches and Cloud-specific networking details.
- 🧪 Tests: N/A
- 📁 Files: `backend/main.py`, `AI-BOOK/docusaurus.config.ts`
- 🧠 Reflection: Default Docusaurus configurations often contain placeholders that users forget to update. The ingestion script should handle these gracefully if the actual deployment URL is known.
