---
id: 9
title: Backend Translation Integration
stage: tasks
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-textbook
branch: 01-physical-ai-textbook
user: user
command: /sp.tasks
labels: ["ui", "localization", "translation", "backend", "api"]
links:
  spec: ../specs/01-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - src/components/TranslationWrapper.tsx
  - specs/01-physical-ai-textbook/tasks-backend-translation-integration.md
tests: []
---

# Backend Translation Integration

## User Input

```
"Connect Frontend Translation to Backend API"
Please update `src/components/TranslationWrapper.tsx` to fetch translations from the Python Backend.

1. MODIFY `TranslationWrapper.tsx`:
   - Add a function `translateHeaderWithAI()` inside the component.
   - Logic:
     - Select the `h1` tag (Page Title).
     - If `isUrdu` is becoming true:
       - Send `h1.innerText` to `POST http://localhost:8000/api/translate`.
       - JSON Body: `{ "text": "Original Title..." }`
       - Replace `h1.innerText` with the response: `data.translated_text`.
     - If `isUrdu` is becoming false:
       - Revert `h1` to the stored original text.

2. KEEP DICTIONARY:
   - Keep the existing TreeWalker dictionary logic for the rest of the page (body text) so the page doesn't look empty.

3. GOAL:
   - Hybrid Approach: Header uses AI (Backend), Body uses Dictionary (Fast Local).

Go ahead and update the file.
```

## Response

Successfully updated the TranslationWrapper component to integrate with the backend API:

1. Added translateHeaderWithAI() function that connects to the Python backend
2. Implemented logic to select h1 tags for AI-powered translation
3. Added API calls to fetch translations from the backend service
4. Maintained existing dictionary-based translation for body content
5. Implemented fallback mechanism when backend API is unavailable
6. Created a hybrid approach where headers use AI translation and body uses local dictionary

## Outcome

The translation system now features a hybrid approach with AI-powered header translations from the backend and fast local dictionary translations for body content, providing both quality and performance.

## Evaluation

The implementation successfully creates a hybrid translation system that leverages both backend AI capabilities for headers and fast local dictionary for body content, with proper error handling and fallback mechanisms.