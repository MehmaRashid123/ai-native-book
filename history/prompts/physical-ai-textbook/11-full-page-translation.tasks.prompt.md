---
id: 11
title: Full Page Translation Implementation
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
  - specs/01-physical-ai-textbook/tasks-full-page-translation.md
tests: []
---

# Full Page Translation Implementation

## User Input

```
"Enable Full Page AI Translation"
Please update `src/components/TranslationWrapper.tsx` to translate ALL text content via API.

1. UPDATE LOGIC:
   - Rename `translateHeaderWithAI` to `translatePageWithAI`.
   - Change the selector from `'h1'` to `'h1, h2, h3, p, li'`.
   - **Optimization:** To prevent crashing the browser with 100 requests:
     - Select all elements.
     - Filter out empty ones or very short strings (length < 3).
     - Use `Promise.all` to send requests in batches (or just loop through the first 20 elements for the demo safety).

2. CODE ADJUSTMENT:
   - Inside the loop:
     - Send `element.innerText` to `POST /api/translate`.
     - Replace `element.innerText` with `response.translated_text`.
     - Save the original text in a `data-original-text` attribute on the HTML element itself (cleaner than React state for DOM manipulation).

3. RESTORE LOGIC:
   - When switching back to English, check if `element.getAttribute('data-original-text')` exists.
   - If yes, restore that text.

Go ahead and update the component.
```

## Response

Successfully updated the TranslationWrapper component with full page AI translation:

1. Renamed function from `translateHeaderWithAI` to `translatePageWithAI`
2. Updated selector from `'h1'` to `'h1, h2, h3, p, li'` for broader coverage
3. Implemented optimization limiting processing to first 20 elements to prevent excessive API requests
4. Added filtering for empty or very short strings (length < 3)
5. Implemented data-original-text attribute for original text storage and restoration
6. Added proper restore logic when switching back to English
7. Maintained fallback dictionary translation for non-targeted elements
8. Added error handling with fallback to dictionary translation if API fails

## Outcome

The TranslationWrapper now translates full page content (headers, paragraphs, list items) using the backend AI service with proper optimization and fallback mechanisms.

## Evaluation

The implementation successfully enables full page AI translation while maintaining performance and providing proper error handling and fallback mechanisms.