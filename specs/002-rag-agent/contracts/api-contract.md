# API Contracts: Agent-Based RAG Retrieval for Embedded Book Chatbot

**Feature**: Agent-Based RAG Retrieval for Embedded Book Chatbot
**Date**: 2025-12-26
**Branch**: 002-rag-agent

## Agent Interface Contract

### Query Endpoint
- **Method**: POST
- **Path**: `/query`
- **Request Body**:
  ```json
  {
    "query": "string (required) - The user's question about the book content"
  }
  ```
- **Response**:
  ```json
  {
    "response": "string - The agent's response based on book content",
    "source_passages": "array of strings - The passages used to generate the response",
    "status": "string - 'success', 'not_found', or 'error'"
  }
  ```
- **Success Response (200)**: When the query is processed successfully
- **Error Response (400)**: When the query is malformed
- **Error Response (500)**: When there's an internal error

## Tool Contract: Qdrant Retrieval

### retrieve_from_qdrant(query: str) -> List[str]
- **Input**: Query string to search in the Qdrant collection
- **Output**: List of relevant passages from the book
- **Behavior**: Returns empty list if no relevant content is found

## Agent Contract: Book Q&A Agent

### Input
- User query as natural language text

### Processing
- Calls retrieve_from_qdrant with the user query
- Uses retrieved passages as context for response generation
- If no passages are retrieved, returns "Not found in the book"

### Output
- Response grounded in retrieved passages
- "Not found in the book" when no relevant content exists