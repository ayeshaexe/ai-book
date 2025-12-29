# Quickstart Guide: Agent-Based RAG Retrieval for Embedded Book Chatbot

**Feature**: Agent-Based RAG Retrieval for Embedded Book Chatbot
**Date**: 2025-12-26
**Branch**: 002-rag-agent

## Prerequisites

- Python 3.11+
- OpenRouter API key
- Access to Qdrant instance with `book_embeddings` collection

## Setup

1. **Install dependencies**:
   ```bash
   pip install openai-agents qdrant-client python-dotenv
   ```

2. **Set up environment variables**:
   Create a `.env` file with:
   ```env
   OPENROUTER_API_KEY=your_openrouter_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key  # if required
   ```

## Usage

1. **Run the RAG agent**:
   ```bash
   cd backend
   python agent.py
   ```

2. **Query the agent**:
   The agent will accept queries and respond based on the book content.

## Configuration

- The agent uses OpenRouter API with base URL: `https://openrouter.ai/api/v1`
- The agent connects to the Qdrant collection named `book_embeddings`
- Default model is configured for OpenRouter compatibility

## Testing

Run the following to test basic functionality:

```bash
python -c "
from agent import query_book
response = query_book('What is the main topic of the book?')
print(response)
"
```

## Troubleshooting

- If you get API key errors, verify your OpenRouter API key is set correctly
- If Qdrant connection fails, check your Qdrant URL and credentials
- If no results are returned, verify the `book_embeddings` collection exists and has content