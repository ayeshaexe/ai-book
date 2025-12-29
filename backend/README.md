# Book RAG API and Ingestion Pipeline

This project implements both a FastAPI backend service for querying book content using a Retrieval-Augmented Generation (RAG) agent and a complete pipeline for ingesting content from a book website, generating embeddings, and storing them in Qdrant.

## Features

### RAG API Service
- Query endpoint that accepts questions about book content
- Returns structured responses with answers, confidence scores, and source references
- Integration with RAG agent for accurate responses
- CORS support for frontend integration
- Proper error handling and validation

### Ingestion Pipeline
- Fetch and parse sitemap to discover all book pages
- Extract clean text content from each page
- Chunk text into semantically meaningful segments
- Generate embeddings using Cohere models
- Store embeddings with metadata in Qdrant
- Verify retrieval with sample queries

## Prerequisites

- Python 3.9+
- `uv` package manager (optional, pip works too)
- Cohere API key
- Qdrant Cloud account and API key
- OpenRouter API key (for RAG agent)

## Setup

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd book-ai/backend
   ```

2. **Create a virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   # Or if using uv:
   uv pip install requests beautifulsoup4 lxml cohere qdrant-client python-dotenv fastapi uvicorn
   ```

4. **Set up environment variables**
   ```bash
   cp .env.example .env  # if you have an example file
   # Or edit the existing .env file to add your API keys:
   ```

   Update the `.env` file with your actual API keys:
   ```env
   # Environment variables for Book Ingestion Pipeline and RAG API

   # Cohere API Configuration
   COHERE_API_KEY=your_cohere_api_key_here

   # OpenRouter API Configuration (for RAG agent)
   OPENROUTER_API_KEY=your_openrouter_api_key_here

   # Qdrant Configuration
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here

   # Book Source Configuration
   BOOK_SITEMAP_URL=https://ai-book-hackathon-mu.vercel.app/sitemap.xml
   ```

5. **Run the RAG API service**
   ```bash
   cd backend
   uvicorn frontend:app --reload --port 8000
   ```

## Usage

### RAG API Service

The FastAPI service provides endpoints for querying book content:

1. **Start the API server:**
   ```bash
   uvicorn frontend:app --reload --port 8000
   ```

2. **Query the book content:**
   ```bash
   curl -X POST "http://localhost:8000/query" \
        -H "Content-Type: application/json" \
        -d '{"query": "What is this book about?"}'
   ```

3. **API Documentation:**
   - Interactive documentation: http://localhost:8000/docs
   - Alternative documentation: http://localhost:8000/redoc

### Ingestion Pipeline

1. **Run the complete ingestion pipeline:**
   ```bash
   python main.py
   ```

2. **Or run individual components for testing:**
   ```bash
   # Test sitemap parsing
   python sitemap_parser.py

   # Test content extraction
   python content_extractor.py

   # Test text chunking
   python text_chunker.py
   ```

## Project Structure

```
backend/
├── frontend.py             # FastAPI service for RAG agent integration
├── example_client.py       # Example client code for API interaction
├── test_communication.py   # Test script for client-server communication
├── main.py                 # Main entry point for the ingestion pipeline
├── requirements.txt        # Python dependencies
├── .env                   # Environment variables
├── sitemap_parser.py      # Sitemap fetching and parsing
├── content_extractor.py   # Web content extraction and cleaning
├── text_chunker.py        # Text chunking utilities
├── cohere_embeddings.py   # Cohere API integration
├── qdrant_storage.py      # Qdrant storage and retrieval
├── ingestion_pipeline.py  # Main orchestration logic
├── utils.py              # Utility functions (retry logic, etc.)
└── README.md             # This file
```

## Configuration

- **Sitemap URL**: Update `BOOK_SITEMAP_URL` in `.env` to point to your book's sitemap
- **Cohere Model**: By default uses `embed-multilingual-v3.0`; can be changed in the code
- **Qdrant Collection**: By default uses `book_embeddings` collection

## How It Works

1. **Discovery**: Fetch and parse the sitemap to get all book page URLs
2. **Extraction**: For each page, extract clean text content while preserving semantic structure
3. **Chunking**: Split content into semantically meaningful segments
4. **Embedding**: Generate vector embeddings using Cohere's embedding model
5. **Storage**: Store embeddings with metadata (URL, title, chunk index) in Qdrant
6. **Verification**: Test retrieval with sample queries to ensure quality

## Error Handling

The pipeline includes robust error handling with:
- Retry logic for network requests
- Graceful degradation when individual pages fail
- Detailed logging for debugging
- Validation of embedding generation and storage

## Sample Output

When running the pipeline, you'll see logs like:
```
Starting Book Website Ingestion and Embedding Pipeline
Step 1: Running ingestion pipeline...
INFO - sitemap_parser - Found 15 URLs in sitemap
INFO - content_extractor - Extracted content from https://example.com/chapter1
INFO - text_chunker - Content chunked into 8 chunks
INFO - cohere_embeddings - Generated embeddings for 120 chunks
INFO - qdrant_storage - Successfully stored 120 embeddings in collection 'book_embeddings'
Step 2: Running verification tests...
INFO - qdrant_storage - Collection 'book_embeddings' has 120 points
INFO - __main__ - ✓ Search test passed - found 3 relevant results
🎉 Book ingestion pipeline completed successfully!
```

## Security

- Store API keys in `.env` file, which is git-ignored
- Never commit API keys to version control
- Use environment variables for configuration

## Troubleshooting

- **Rate Limiting**: The pipeline includes backoff logic, but you may need to adjust if getting rate limited
- **Memory Issues**: For large books, the pipeline may need to be modified to process in smaller batches
- **Content Extraction**: If content extraction isn't working well, you may need to adjust selectors in `content_extractor.py`

## Next Steps

Once the pipeline completes successfully, the embeddings are ready for use in a RAG application. You can:
- Build a search interface that queries the Qdrant collection
- Create a chatbot that retrieves relevant content based on user queries
- Build an API to serve the embeddings to downstream applications