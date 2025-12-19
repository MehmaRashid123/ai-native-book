---
id: "006"
title: "remove-mantine-and-fix-crashes"
stage: "red"
date: "2025-12-19"
surface: "agent"
model: "gemini"
feature: "docusaurus-sci-fi-ui-redesign"
branch: "trunk"
user: "user"
command: "replace"
labels: ["bugfix", "docusaurus", "crash", "mantine"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - AI-BOOK/src/theme/Root.tsx
 - AI-BOOK/src/pages/index.tsx
 - AI-BOOK/src/pages/index.module.css
tests: []
---

# Prompt
This page crashed
Try again
Element type is invalid: expected a string (for built-in components) or a class/function (for composite components) but got: undefined. You likely forgot to export your component from the file it's defined in, or you might have mixed up default and named imports.

Check the render method of `Layout`.

# Response
Identified that the project was using `@mantine/core` components in `Root.tsx` and `pages/index.tsx`, but `@mantine/core` was not listed in `package.json` and its use violated the "No external UI frameworks" constraint.
1. Removed Mantine from `src/theme/Root.tsx`.
2. Replaced Mantine components (`Button`, `Card`, `Text`) in `src/pages/index.tsx` with standard HTML/Docusaurus elements.
3. Added custom styling in `src/pages/index.module.css` to maintain the sci-fi aesthetic without Mantine.
This should resolve the "Element type is invalid" crash caused by undefined Mantine components.
