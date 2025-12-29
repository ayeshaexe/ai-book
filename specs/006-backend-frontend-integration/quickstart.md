# Quickstart: Backend-Frontend Integration

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to OpenRouter API key
- Access to Qdrant vector database with book embeddings

## Setup

1. **Install dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv
   ```

2. **Set up environment variables**:
   Create a `.env` file with the following:
   ```env
   OPENROUTER_API_KEY=your_openrouter_api_key
   QDRANT_HOST=localhost
   QDRANT_PORT=6333
   ```

3. **Ensure the RAG agent is accessible**:
   Make sure `backend/agent.py` exists and contains the RAGAgent class with the `process_query_sync` method.

## Running the Service

1. **Start the FastAPI server**:
   ```bash
   cd backend
   uvicorn frontend:app --reload --port 8000
   ```

2. **Test the endpoint**:
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"query": "What is physical AI?"}'
   ```

## API Usage

### Query Endpoint
- **URL**: `POST /query`
- **Request Body**:
  ```json
  {
    "query": "Your question here"
  }
  ```
- **Response**:
  ```json
  {
    "answer": "The answer to your question...",
    "confidence": 0.85,
    "sources": [
      {
        "content": "Source text...",
        "metadata": {}
      }
    ]
  }
  ```

## Example Client Code

### JavaScript/Fetch Example
```javascript
async function queryBookContent(question) {
  try {
    const response = await fetch('http://localhost:8000/query', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: question
      })
    });

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error querying book content:', error);
    throw error;
  }
}

// Usage
queryBookContent("What are the main concepts of physical AI?")
  .then(result => console.log(result.answer));
```

## Troubleshooting

- **500 Error**: Check that the RAG agent is properly configured and accessible
- **404 Error**: Verify the endpoint URL and ensure the server is running
- **Connection Error**: Confirm Qdrant database is running and accessible
- **API Key Error**: Verify the OpenRouter API key is valid and properly set in environment variables