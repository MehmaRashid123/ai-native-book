# Data Model

## Entities

### Document Chunk
Represents a piece of text extracted from the book documentation, ready for embedding.

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | String (UUID) | Unique identifier for the chunk. |
| `source_url` | String | The URL of the page where the text was found. |
| `page_title` | String | The title of the page (e.g., from HTML `<title>` or `h1`). |
| `text_content` | String | The actual text content of the chunk. |
| `token_count` | Integer | (Optional) Approximate number of tokens in the chunk. |

### Vector Record
The entity stored in Qdrant.

| Field | Type | Description |
|-------|------|-------------|
| `id` | String (UUID) | Matches the `chunk_id`. |
| `vector` | List[Float] | The embedding vector returned by Cohere. |
| `payload` | JSON Object | Stores the metadata (`source_url`, `page_title`, `text_content`). |

## Qdrant Collection Schema

- **Collection Name**: `rag_embedding`
- **Vector Size**: 1024 (for `embed-english-v3.0`)
- **Distance Metric**: Cosine (standard for text embeddings)
