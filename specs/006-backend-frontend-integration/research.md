# Research: Backend-Frontend Integration

## Decision: FastAPI for Backend Service
**Rationale**: FastAPI is an excellent choice for this project because it provides automatic API documentation, type validation, and high performance. It's ideal for creating REST APIs that integrate with AI services like the RAG agent. The framework also has excellent async support which is important for AI query processing.

**Alternatives considered**:
- Flask: Simpler but lacks automatic documentation and type validation
- Django: More complex than needed for this API-only service
- Express.js: Would require switching to Node.js ecosystem

## Decision: Integration Pattern with Existing RAG Agent
**Rationale**: Rather than duplicating functionality, we'll import and use the existing RAG agent from `backend/agent.py`. This maintains consistency with existing code and leverages the already-implemented vector storage and retrieval logic. The FastAPI service will act as a thin API layer over the RAG agent.

**Alternatives considered**:
- Reimplementing RAG functionality: Would create code duplication and maintenance overhead
- Using a separate service: Would add network complexity for a local integration

## Decision: API Endpoint Structure
**Rationale**: The `/query` POST endpoint will accept JSON with a "query" field and return structured responses with answer, confidence, and sources. This follows REST conventions and provides all necessary information for frontend consumption.

**Request format**:
```json
{
  "query": "What is the main concept of this book?"
}
```

**Response format**:
```json
{
  "answer": "The main concept is...",
  "confidence": 0.85,
  "sources": [
    {
      "content": "Source text...",
      "metadata": {...}
    }
  ]
}
```

## Decision: Environment Configuration
**Rationale**: The service will use the same environment variables as the existing RAG agent, particularly the OpenRouter API key. This ensures compatibility and avoids duplicating configuration.

**Required environment variables**:
- `OPENROUTER_API_KEY`: For accessing the OpenRouter API
- `QDRANT_HOST`: For accessing the Qdrant vector database
- `QDRANT_PORT`: For accessing the Qdrant vector database

## Decision: Error Handling Strategy
**Rationale**: The API will return appropriate HTTP status codes and meaningful error messages for different failure scenarios:
- 400 Bad Request: For malformed requests
- 422 Unprocessable Entity: For validation errors
- 500 Internal Server Error: For system failures
- 503 Service Unavailable: If the RAG agent service is unavailable

## Decision: Testing Approach
**Rationale**: We'll use pytest for testing the API endpoints, with both unit tests for individual functions and integration tests for the full API flow. Mocking will be used for external dependencies like the RAG agent to ensure tests are reliable and fast.