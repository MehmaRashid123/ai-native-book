---
id: 005
title: fix-auth-server-cors-and-database
stage: green
date: 2025-12-19
surface: agent
model: gemini
feature: auth-personalization
branch: 004-auth-personalization
user: user
command: none
labels: ["bugfix", "betterauth", "hono", "postgresql", "cors"]
links:
  spec: specs/004-auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - auth-server/src/auth.ts
 - auth-server/src/index.ts
 - auth-server/.env
 - auth-server/package.json
 - AI-BOOK/src/theme/Root.tsx
tests: []
---

# Prompt
postgresql://neondb_owner:npg_X2GaZbPFj1LN@ep-autumn-night-a4722g1t-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

Access to fetch at 'http://localhost:3001/api/auth/get-session' from origin 'http://localhost:3000' has been blocked by CORS policy...
GET http://localhost:3001/api/auth/get-session net::ERR_FAILED 404 (Not Found)
signup:1 Access to fetch at 'http://localhost:3001/api/auth/sign-up/email' from origin 'http://localhost:3000' has been blocked by CORS policy...

# Response
1. **Database Migration**: Switched from SQLite to PostgreSQL (Neon) using the provided connection string. Installed `pg` driver and updated `auth.ts`.
2. **CORS Fix**: Refined Hono CORS middleware to correctly handle preflight requests and origins.
3. **404 Resolution**: Updated the frontend `authUrl` in `Root.tsx` to include the required `/api/auth` prefix, matching the server's endpoint structure.
4. **Development Workflow**: Added a `dev` script to `auth-server/package.json` for easier local execution.
