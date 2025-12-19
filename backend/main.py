import os
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from langchain_text_splitters import RecursiveCharacterTextSplitter
from dotenv import load_dotenv
import uuid
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("pipeline.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Initialize clients
try:
    co = cohere.ClientV2(COHERE_API_KEY) if COHERE_API_KEY else None
    
    if QDRANT_URL:
        # ABSOLUTE SIMPLEST INITIALIZATION - No cleaning, use .env exactly
        # If QDRANT_URL is 'https://xxxx.qdrant.io', this is the standard way.
        qdrant = QdrantClient(
            url=QDRANT_URL if QDRANT_URL.startswith("http") else f"https://{QDRANT_URL}",
            api_key=QDRANT_API_KEY
        )
        logger.info(f"Connecting to Qdrant at: {QDRANT_URL}")
    else:
        qdrant = None
        
    if not co or not qdrant:
        logger.warning("API clients not fully initialized. Check your .env file.")
except Exception as e:
    logger.error(f"Failed to initialize clients: {e}")
    co = None
    qdrant = None

def test_connections():
    """Test if we can actually reach the services."""
    logger.info("Testing connections...")
    
    # 1. Test Cohere
    try:
        if co:
            co.chat(model="command-r-08-2024", messages=[{"role": "user", "content": "hello"}])
            logger.info("Cohere connection: OK")
        else:
            logger.error("Cohere API key missing.")
            return False
    except Exception as e:
        logger.error(f"Cohere test failed: {e}")
        return False
            
    # 2. Test Qdrant
    try:
        if qdrant:
            qdrant.get_collections()
            logger.info("Qdrant connection: OK")
            return True
        else:
            logger.error("Qdrant URL/API key missing.")
            return False
    except Exception as e:
        logger.error(f"Qdrant test failed: {e}")
        logger.info("TIP: Check if your QDRANT_URL in .env is the API Endpoint, not the Dashboard URL.")
        logger.info("Example: https://xxxx-xxxx-xxxx.europe-west3-0.gcp.cloud.qdrant.io:6333")
        return False

# --- FastAPI Implementation for Chatbot ---
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    query: str

@app.post("/chat")
async def chat_endpoint(request: QueryRequest):
    if not co or not qdrant:
        raise HTTPException(status_code=500, detail="Backend services not initialized")
    
    results = search(request.query)
    context = "\n".join([res.payload.get("text", "") for res in results])
    
    if not context:
        return {"answer": "I couldn't find any relevant information in the book. Try asking about ROS 2 or Simulation."}

    try:
        response = co.chat(
            model="command-r-08-2024",
            messages=[
                {"role": "system", "content": f"You are a helpful assistant for the Physical AI book. Answer based on this context:\n\n{context}"},
                {"role": "user", "content": request.query}
            ]
        )
        # Deduplicate sources
        sources = list(set([res.payload.get("url") for res in results if res.payload.get("url")]))
        return {"answer": response.message.content[0].text, "sources": sources}
    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail="Failed to generate response")

# --- Pipeline Functions ---

def get_all_urls(sitemap_url):
    logger.info(f"Fetching sitemap: {sitemap_url}")
    try:
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'xml')
        urls = [loc.text for loc in soup.find_all('loc')]
        
        base_deploy_url = "https://ai-native-book-psi.vercel.app"
        placeholder_domain = "your-docusaurus-site.example.com"
        
        return [u.replace(f"https://{placeholder_domain}", base_deploy_url).replace(f"http://{placeholder_domain}", base_deploy_url) for u in urls]
    except Exception as e:
        logger.error(f"Sitemap error: {e}")
        return []

def extract_text_from_url(url):
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')
        for s in soup(["script", "style"]): s.decompose()
        main = soup.find('main') or soup.find('article') or soup.body
        return main.get_text(separator=' ', strip=True) if main else ""
    except Exception as e:
        logger.error(f"Extraction error for {url}: {e}")
        return ""

def chunk_text(text):
    return RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=100).split_text(text)

def embed(text_chunks):
    if not co: return None
    try:
        response = co.embed(texts=text_chunks, model="embed-english-v3.0", input_type="search_document", embedding_types=["float"])
        return response.embeddings.float
    except Exception as e:
        logger.error(f"Embedding error: {e}")
        return None

def create_collection(collection_name):
    if not qdrant: return
    try:
        cols = qdrant.get_collections().collections
        if not any(c.name == collection_name for c in cols):
            logger.info(f"Creating collection: {collection_name}")
            qdrant.create_collection(collection_name=collection_name, vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE))
    except Exception as e:
        logger.error(f"Collection error: {e}")

def save_chunk_to_qdrant(collection_name, chunks, embeddings, url):
    if not qdrant or not embeddings: return
    try:
        points = [models.PointStruct(id=str(uuid.uuid4()), vector=v, payload={"text": c, "url": url}) for c, v in zip(chunks, embeddings)]
        qdrant.upsert(collection_name=collection_name, points=points)
    except Exception as e:
        logger.error(f"Upsert error: {e}")

def search(query, collection_name="rag_embedding", top_k=3):
    if not co or not qdrant: return []
    try:
        ev = co.embed(texts=[query], model="embed-english-v3.0", input_type="search_query", embedding_types=["float"]).embeddings.float[0]
        # query_points is the correct method for v1.10+ of qdrant-client
        results = qdrant.query_points(
            collection_name=collection_name,
            query=ev,
            limit=top_k
        ).points
        return results
    except Exception as e:
        logger.error(f"Search error: {e}")
        return []

def main():
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "ingest"
    
    if mode == "test":
        test_connections()
        return

    if mode == "server":
        import uvicorn
        logger.info("Starting API Server...")
        uvicorn.run(app, host="0.0.0.0", port=8000)
        return

    # Ingestion flow
    if not test_connections():
        logger.error("Connection failed. Check your URL and API keys in .env")
        return

    deploy_url = "https://ai-native-book-psi.vercel.app/"
    sitemap_url = f"{deploy_url}sitemap.xml"
    collection_name = "rag_embedding"
    
    create_collection(collection_name)
    urls = get_all_urls(sitemap_url)
    
    for url in urls:
        if any(x in url for x in ["/tags", "/blog/archive", "/search", "/404"]): continue
        text = extract_text_from_url(url)
        if not text: continue
        chunks = chunk_text(text)
        logger.info(f"Processing {url} ({len(chunks)} chunks)")
        embeddings = embed(chunks)
        if embeddings:
            save_chunk_to_qdrant(collection_name, chunks, embeddings, url)
        time.sleep(0.5)
    
    logger.info("Ingestion complete.")

if __name__ == "__main__":
    main()
