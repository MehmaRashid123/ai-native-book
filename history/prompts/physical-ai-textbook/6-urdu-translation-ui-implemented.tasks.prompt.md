---
id: 6
title: Urdu Translation UI Implemented
stage: tasks
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-textbook
branch: 01-physical-ai-textbook
user: user
command: /sp.tasks
labels: ["ui", "localization", "translation", "urdu"]
links:
  spec: ../specs/01-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - src/components/TranslationWrapper.tsx
  - src/theme/Root.js
  - specs/01-physical-ai-textbook/tasks-urdu-translation.md
tests: []
---

# Urdu Translation UI Implemented

## User Input

```
"Implement Urdu Translation UI"
Please create the Urdu Translation feature for the frontend.

1. COMPONENT:
   - Create a file `src/components/TranslationWrapper.tsx`.
   - It should have a button: "🌐 Translate to Urdu".
   - **Logic:** When clicked, use a simple Javascript mapping to replace common English headings with Urdu (Mock Translation for Demo).
     - Example Mappings:
       - "Introduction" -> "Taaruf (تعارف)"
       - "Module" -> "Hissa (حصہ)"
       - "Overview" -> "Jaiza (جائزہ)"
       - "Physical AI" -> "Physical AI (جسمانی مصنوعی ذہانت)"
       - "Robotics" -> "Robotics (روبوٹکس)"
       - "Hardware" -> "Hardware (ہارڈ ویئر)"
   - Store the state (isUrdu) so the user can toggle it back to English.

2. INTEGRATION:
   - Create/Update `src/theme/Root.js`.
   - Wrap the entire application with this `TranslationWrapper` so the button is available globally or inject it at the top of documentation pages.

Go ahead and create this component.
```

## Response

Successfully implemented the Urdu Translation UI feature with:

1. Created `src/components/TranslationWrapper.tsx` with:
   - Toggle button "🌐 Translate to Urdu"
   - Translation mappings for common terms
   - State management for isUrdu toggle
   - DOM translation functionality
   - Preservation of text case and exclusion of code elements

2. Created `src/theme/Root.js` to wrap the entire application with TranslationWrapper

The feature is now available globally across all pages with a sticky translation control button.

## Outcome

The Urdu translation feature is fully implemented with a toggle button that translates common English terms to Urdu with romanized pronunciation. The translation system is integrated at the application root level for global availability.

## Evaluation

The implementation successfully meets all requirements with proper component structure, state management, and global integration. The translation functionality preserves original formatting while providing appropriate Urdu translations with romanized pronunciation.