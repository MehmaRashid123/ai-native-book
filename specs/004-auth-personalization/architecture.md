# Architecture: Authentication & Personalization

## Overview
The system uses a decoupled architecture where identity is managed by a dedicated Node.js `auth-server` using BetterAuth, and the Docusaurus frontend consumes this session to tailor content.

## Components

### 1. Auth Server (Hono + BetterAuth)
- **Role**: Handles signup, signin, and session verification.
- **Persistence**: SQLite (better-sqlite3).
- **Schema Extension**: The `user` table includes `operating_system`, `experience_level`, and `hardware_access`.

### 2. Frontend Identity Layer (React Context)
- **AuthProvider**: Wraps the Docusaurus site at the root level.
- **BetterAuth Client**: Synchronizes session state with the auth server.
- **useAuth Hook**: Provides easy access to user profile data across components.

### 3. Personalization Engine (Conditional Rendering)
- **PersonalizedContent Component**: A wrapper that uses `useAuth()` to check user attributes.
- **Matching Logic**:
    - If user OS matches prop -> Show content.
    - If user Level matches prop -> Show content.
    - Otherwise -> Show fallback or nothing.

## Data Flow
1. User signs up on `/signup`.
2. Data is sent to Hono server.
3. BetterAuth creates user and session in SQLite.
4. Session cookie/token is returned to browser.
5. `AuthProvider` fetches session on load.
6. `PersonalizedContent` reads session and updates UI.
