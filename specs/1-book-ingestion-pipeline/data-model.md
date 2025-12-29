# Data Model: Book Website Ingestion and Embedding Pipeline

**Feature**: 1-book-ingestion-pipeline
**Created**: 2025-12-25
**Status**: Design

## Entity: BookContent

### Description
Represents the extracted text from book pages, including the raw content and source information.

### Fields
- **id** (string, required): Unique identifier for the content chunk (UUID format)
- **url** (string, required): Source URL of the original page
- **title** (string, required): Page title extracted from HTML
- **content** (string, required): Clean text content of the chunk
- **chunk_index** (integer, required): Sequential index for content chunks from the same page
- **created_at** (datetime, required): Timestamp of when the content was ingested

### Validation Rules
- `url` must be a valid URL format
- `title` must be 1-500 characters
- `content` must be 1-10000 characters (to ensure reasonable chunk sizes)
- `chunk_index` must be >= 0
- `created_at` must be in ISO 8601 format

### Relationships
- One-to-many with Embedding (one content chunk can have one embedding)

## Entity: Embedding

### Description
Represents the vector representation of a content chunk, with associated metadata.

### Fields
- **vector_id** (string, required): Unique identifier for the vector in Qdrant
- **content_id** (string, required): Reference to BookContent.id
- **vector** (array<float>, required): The embedding vector (1024-dimensional for Cohere multilingual model)
- **metadata** (object, required): JSON object containing {url, title, chunk_index}
- **created_at** (datetime, required): Timestamp of when the embedding was generated

### Validation Rules
- `vector_id` must be unique
- `content_id` must reference an existing BookContent.id
- `vector` must have exactly 1024 dimensions
- `metadata` must contain required fields: url, title, chunk_index
- `created_at` must be in ISO 8601 format

### Relationships
- Many-to-one with BookContent (many embeddings reference one content chunk)

## Entity: SourceMetadata

### Description
Contains information about the original source (URL, title, chunk index) for each embedding.

### Fields
- **url** (string, required): The original page URL
- **title** (string, required): The page title
- **chunk_index** (integer, required): Index of the content chunk in the original page
- **ingestion_timestamp** (datetime, required): When the page was processed

### Validation Rules
- `url` must be a valid URL format
- `title` must be 1-500 characters
- `chunk_index` must be >= 0
- `ingestion_timestamp` must be in ISO 8601 format

### Relationships
- This entity is primarily stored as metadata within the Embedding entity
- Used for retrieval and attribution purposes

## Collection Schema (Qdrant)

### Collection Name
`book_embeddings`

### Vector Configuration
- Size: 1024 (matching Cohere embed-multilingual-v3.0 output)
- Distance: Cosine

### Payload Schema
```
{
  "url": "string (indexed)",
  "title": "string (indexed)",
  "chunk_index": "integer (indexed)",
  "content_preview": "string"
}
```

### Indexes
- Index on `url` for filtering by source
- Index on `title` for filtering by document title
- Index on `chunk_index` for ordering content chunks