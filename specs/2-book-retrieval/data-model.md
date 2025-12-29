# Data Model: Book Data Retrieval & Pipeline Verification

## Entities

### BookContentChunk
**Description**: Represents a segment of book text with associated metadata
**Fields**:
- `id` (string): Unique identifier for the chunk in Qdrant
- `url` (string): The source URL where this content originated
- `chunk_index` (integer): Sequential index of this chunk within the source document
- `text` (string): The actual text content of the chunk
- `vector` (list[float]): Embedding vector representation of the text content
- `metadata` (dict): Additional metadata fields (title, page number, etc.)

### QdrantCollection
**Description**: The vector database storage containing the ingested book content
**Fields**:
- `name` (string): Name of the collection (`book_embeddings`)
- `vectors_count` (integer): Total number of vector records in the collection
- `points` (list[BookContentChunk]): Collection of all stored content chunks

### URLValidationResult
**Description**: Result of comparing sitemap URLs with stored URLs
**Fields**:
- `expected_urls` (set[string]): URLs from the sitemap that should be in Qdrant
- `stored_urls` (set[string]): URLs that are actually stored in Qdrant
- `missing_urls` (set[string]): URLs from sitemap that are not found in Qdrant
- `extra_urls` (set[string]): URLs in Qdrant that are not in the sitemap
- `completeness_percentage` (float): Percentage of expected URLs that are present

### RetrievalQuery
**Description**: Parameters for executing a similarity search query
**Fields**:
- `query_text` (string): Text to use for similarity search
- `top_k` (integer): Number of results to return (default: 5)
- `query_vector` (list[float]): Vector representation of the query text
- `filter_conditions` (dict): Optional filters to apply to the search

### RetrievalResult
**Description**: Result of a similarity search query
**Fields**:
- `query_text` (string): Original query text
- `results` (list[BookContentChunk]): Retrieved content chunks, ranked by relevance
- `execution_time` (float): Time taken to execute the query in seconds
- `relevance_scores` (list[float]): Similarity scores for each result