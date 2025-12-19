---
id: "003"
title: "fix-layout-crash"
stage: "red"
date: "2025-12-19"
surface: "agent"
model: "gemini"
feature: "docusaurus-sci-fi-ui-redesign"
branch: "trunk"
user: "user"
command: "replace"
labels: ["bugfix", "docusaurus", "crash"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - AI-BOOK/src/theme/Layout/index.tsx
tests: []
---

# Prompt
This page crashed
Try again
Element type is invalid: expected a string (for built-in components) or a class/function (for composite components) but got: undefined. You likely forgot to export your component from the file it's defined in, or you might have mixed up default and named imports.

Check the render method of `Layout`.

# Response
Identified incorrect import in `src/theme/Layout/index.tsx`.
The file was importing `Navbar` and `Footer` from `@docusaurus/theme-common/internal`, which likely resulted in `undefined` values.
Replaced with standard theme imports:
```typescript
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
```
This should resolve the "Element type is invalid" crash.
