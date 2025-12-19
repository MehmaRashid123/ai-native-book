# Quickstart: Authentication & Personalization

## 1. Setup Auth Server
Navigate to the new `auth-server` directory and install dependencies:
```bash
cd auth-server
npm install better-auth better-sqlite3 hono
```

## 2. Configure Environment
Create a `.env` file in `auth-server/`:
```ini
BETTER_AUTH_SECRET=your_secret_key
DATABASE_URL=sqlite.db
```

## 3. Initialize Database
Run the BetterAuth migration command to set up the SQLite schema:
```bash
npx better-auth migrate
```

## 4. Run Development Server
```bash
npm run dev
```

## 5. Frontend Integration
In `AI-BOOK/src/theme/Root.tsx`, wrap the application with the `AuthContext`:
```tsx
import { AuthProvider } from '../components/Auth/AuthContext';

export default function Root({children}) {
  return (
    <AuthProvider authUrl="http://localhost:3001">
      {children}
    </AuthProvider>
  );
}
```
