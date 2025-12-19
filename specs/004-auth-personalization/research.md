# Research: BetterAuth Integration & Personalization

## Technical Context
We need to implement authentication in a Docusaurus project using BetterAuth and use the session data to personalize MDX content.

### Identified Unknowns & Research Tasks
- **Task 1**: Research BetterAuth integration with Docusaurus/React.
- **Task 2**: Investigate storing software/hardware background in BetterAuth user sessions.
- **Task 3**: Determine the best database for a low-friction deployment (e.g., SQLite with a persistent volume or Supabase/PostgreSQL).

## Findings

### 1. BetterAuth + Docusaurus Integration
- **Decision**: Use a dedicated `auth-server` (Node.js/Express or Hono) to host the BetterAuth instance.
- **Rationale**: Docusaurus is primarily a static site generator. While it has client-side React, BetterAuth requires server-side endpoints for session management and database interaction. A small Hono or Express server is the standard way to provide these endpoints.
- **Alternatives**: Using Next.js for the whole project (Rejected: current project is already Docusaurus).

### 2. Custom User Metadata
- **Decision**: Extend the BetterAuth schema to include custom fields (`operating_system`, `experience_level`, `hardware_access`).
- **Rationale**: BetterAuth allows for easy schema extension. Storing these directly in the `user` table ensures they are available in the session object without extra database queries.
- **Implementation**: Use the `additionalFields` property in BetterAuth configuration.

### 3. Database Choice
- **Decision**: **SQLite** (using the `better-sqlite3` adapter) for development and MVP.
- **Rationale**: Zero-config, easy to back up, and sufficient for the initial student base. For production deployment on Render/Hugging Face, we can use a persistent disk or switch to a managed PostgreSQL (Supabase) by changing the adapter.

### 4. Personalization Mechanism
- **Decision**: Create a React Context Provider in `AI-BOOK/src/theme/Root.tsx` to wrap the entire site with the auth state.
- **Rationale**: This allows any component or MDX page to access `useAuth()` and conditionally render content.
- **Component**: `<PersonalizedContent os="linux"> ... </PersonalizedContent>`

## Rationale Summary

| Decision | Rationale |
|----------|-----------|
| Hono Server | Lightweight, fast, and perfect for a standalone auth API. |
| SQLite | Simplest data storage for a textbook project. |
| React Context | Efficiently propagates auth state to the whole documentation tree. |
