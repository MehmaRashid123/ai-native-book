# Quickstart Guide

## Prerequisites
- Python 3.10+
- Qdrant Cloud Account & API Key
- Cohere API Key

## Setup

1. **Navigate to Backend**:
   ```bash
   cd backend
   ```

2. **Initialize Environment**:
   ```bash
   python -m venv .venv
   # On Windows:
   .venv\Scripts\activate
   # On Linux/Mac:
   source .venv/bin/activate
   ```

3. **Install Dependencies**:
   ```bash
   pip install cohere qdrant-client beautifulsoup4 langchain-text-splitters lxml python-dotenv requests
   ```

4. **Configure Environment**:
   Create a `.env` file in `backend/`:
   ```ini
   COHERE_API_KEY=your_cohere_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   ```

## Running the Pipeline

To scrape the website, generate embeddings, and populate Qdrant:

```bash
python main.py
```

Check `pipeline.log` for detailed progress.

## Verification

To test the retrieval system with a sample query:

```bash
python verify_rag.py
```

## Implementation Details
- **Discovery**: Uses `sitemap.xml` from the deployed Vercel site.
- **Scraping**: Uses `BeautifulSoup` to extract text from `<main>` content.
- **Indexing**: Upserts semantic vectors to the `rag_embedding` collection in Qdrant.