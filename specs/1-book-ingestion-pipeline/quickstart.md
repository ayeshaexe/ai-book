# Quickstart Guide: Book Website Ingestion and Embedding Pipeline

**Feature**: 1-book-ingestion-pipeline
**Created**: 2025-12-25

## Overview
This guide will help you set up and run the book website ingestion and embedding pipeline. The pipeline discovers book pages via sitemap, extracts content, generates embeddings using Cohere, and stores them in Qdrant.

## Prerequisites
- Python 3.9 or higher
- `uv` package manager installed
- Cohere API key
- Qdrant Cloud cluster URL and API key

## Setup Instructions

### 1. Clone or navigate to the project directory
```bash
cd your-project-directory
```

### 2. Create the backend directory
```bash
mkdir backend
cd backend
```

### 3. Initialize the project with uv
```bash
uv init
```

### 4. Install dependencies
```bash
uv pip install requests beautifulsoup4 lxml cohere qdrant-client python-dotenv
```

### 5. Set up environment variables
Create a `.env` file in the backend directory with the following content:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
BOOK_SITEMAP_URL=https://ai-book-hackathon-mu.vercel.app/sitemap.xml
```

### 6. Create the main.py file
Create a `main.py` file with the implementation of the ingestion pipeline (see Implementation section below).

## Running the Pipeline

### 1. Execute the ingestion pipeline
```bash
python main.py
```

### 2. Monitor the output
- The pipeline will fetch the sitemap and discover all book pages
- It will extract and clean content from each page
- Generate embeddings using Cohere
- Store embeddings in Qdrant with metadata
- Verify retrieval with sample queries

## Implementation Structure

The main.py file should implement the following components:

### 1. Sitemap Parser
- Fetch and parse the sitemap.xml file
- Extract all book page URLs

### 2. Content Extractor
- Fetch each book page
- Extract main content, remove navigation and other elements
- Clean and structure the text content

### 3. Text Chunker
- Split content into appropriately sized chunks
- Preserve semantic meaning and context

### 4. Embedding Generator
- Use Cohere API to generate embeddings
- Handle API rate limits and errors

### 5. Qdrant Storage
- Create collection if needed
- Store embeddings with metadata
- Verify storage success

### 6. Verification
- Perform sample queries to verify embedding quality
- Validate that retrieved content is relevant

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `COHERE_API_KEY` | Your Cohere API key for embedding generation | Required |
| `QDRANT_URL` | Your Qdrant Cloud cluster URL | Required |
| `QDRANT_API_KEY` | Your Qdrant API key | Required |
| `BOOK_SITEMAP_URL` | URL to the book sitemap | https://ai-book-hackathon-mu.vercel.app/sitemap.xml |

## Troubleshooting

### Common Issues

1. **API Rate Limits**: The pipeline implements exponential backoff for API calls. If you encounter rate limit issues, consider spacing out your runs.

2. **Network Errors**: The pipeline has retry logic for network requests. If pages fail to load, check your internet connection and the target website's availability.

3. **Qdrant Connection**: Ensure your QDRANT_URL and QDRANT_API_KEY are correct. Test the connection separately if needed.

4. **Memory Usage**: Processing large books may require significant memory. Monitor your system resources during execution.

### Verification Steps

1. Check that all pages from the sitemap were processed
2. Verify that embeddings were created in Qdrant
3. Test sample queries to ensure content retrieval works
4. Review logs for any errors or warnings

## Next Steps

After successful execution:
1. The embeddings will be available in Qdrant for downstream RAG applications
2. You can perform semantic searches against the stored content
3. Integrate with your chatbot application for content retrieval