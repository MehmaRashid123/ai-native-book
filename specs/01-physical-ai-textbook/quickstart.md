# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: 01-physical-ai-textbook
**Date**: 2025-12-10

## Overview

This guide provides a quick start for developers to set up and run the Physical AI & Humanoid Robotics Textbook project. The system consists of a Docusaurus frontend with a FastAPI backend for the Vector RAG agent.

## Prerequisites

- Node.js 18+ (for frontend)
- Python 3.11+ (for backend)
- Git
- Docker (optional, for local Qdrant/PostgreSQL)

## Environment Setup

### Frontend (Docusaurus)

1. **Install dependencies**:
   ```bash
   cd frontend
   npm install
   ```

2. **Set environment variables**:
   Create a `.env` file in the frontend directory:
   ```env
   BACKEND_API_URL=http://localhost:8000
   AUTH_ENABLED=true
   ```

3. **Run the development server**:
   ```bash
   npm start
   ```

### Backend (FastAPI)

1. **Create virtual environment**:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Set environment variables**:
   Create a `.env` file in the backend directory:
   ```env
   QDRANT_URL=http://localhost:6333
   DATABASE_URL=postgresql://user:password@localhost:5432/physical_ai_handbook
   AUTH_SECRET=your-super-secret-jwt-key
   ```

4. **Run the backend server**:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

### Database Setup

1. **For Neon PostgreSQL**:
   - Create a Neon project
   - Update the DATABASE_URL in your backend .env file
   - Run migrations: `python -m src.database.migrations`

2. **For Qdrant Vector Database**:
   - Option A: Use Qdrant Cloud (recommended for production)
   - Option B: Run locally with Docker:
     ```bash
     docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant
     ```

## Development Workflow

### Adding New Content

1. Create new markdown files in the `frontend/docs/` directory following the module structure:
   ```
   docs/
   ├── 01-module-1/
   │   ├── 01-intro-ros2.md
   │   ├── 02-python-agents.md
   │   └── 03-urdf.md
   ├── 02-module-2/
   └── ...
   ```

2. Update `frontend/sidebars.ts` to include the new content in the navigation.

3. Run content ingestion to update vector embeddings:
   ```bash
   cd backend
   python -m src.ingestion.ingest_docs
   ```

### Adding New Agent Skills

1. Create a new skill in `backend/src/services/`
2. Register the skill in the agent's skill registry
3. Update the API contract if needed
4. Test the new functionality

### Running Tests

**Frontend tests**:
```bash
npm test
npm run test:e2e  # For end-to-end tests
```

**Backend tests**:
```bash
cd backend
pytest
```

## Key Endpoints

### Frontend (Docusaurus)
- `http://localhost:3000` - Main application
- `http://localhost:3000/api/` - API routes

### Backend (FastAPI)
- `http://localhost:8000/docs` - API documentation
- `http://localhost:8000/api/v1/chat` - Vector agent chat endpoint
- `http://localhost:8000/api/v1/translate` - Translation endpoint
- `http://localhost:8000/api/v1/hardware-check` - Hardware compatibility endpoint

## Common Tasks

### Initialize the Project
```bash
# Clone and set up frontend
git clone <repository>
cd frontend
npm install
npm start

# In another terminal, set up backend
cd backend
pip install -r requirements.txt
uvicorn src.api.main:app --reload
```

### Update Content Embeddings
```bash
cd backend
python -m src.ingestion.ingest_docs
```

### Deploy to Production
1. Build frontend: `npm run build`
2. Deploy to GitHub Pages (configured in docusaurus.config.js)
3. Deploy backend to Vercel (with proper environment variables)

## Troubleshooting

### Frontend Issues
- If Tailwind styles aren't loading: Check that `tailwind.config.js` is properly configured and CSS imports are correct
- If API calls fail: Verify BACKEND_API_URL is set correctly in frontend .env

### Backend Issues
- If Qdrant connection fails: Ensure Qdrant is running and URL is correct in .env
- If authentication fails: Check AUTH_SECRET is set and consistent across services

### Content Issues
- If new content doesn't appear: Verify it's added to sidebars.ts and run the ingestion process
- If search doesn't find content: Re-run the embedding ingestion process