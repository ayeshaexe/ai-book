# Data Model: Agent-Based RAG Retrieval for Embedded Book Chatbot

**Feature**: Agent-Based RAG Retrieval for Embedded Book Chatbot
**Date**: 2025-12-26
**Branch**: 002-rag-agent

## Key Entities

### RAG Agent
- **Name**: String identifier for the agent
- **Instructions**: Text containing the agent's behavior guidelines
- **Tools**: List of function tools available to the agent
- **Model Configuration**: Settings for the LLM to use

### Query
- **Text**: Natural language question from the user
- **Metadata**: Additional context for the query (optional)

### Retrieved Passage
- **Content**: Text content retrieved from the book
- **Score**: Relevance score from vector search
- **Source**: Reference to the original location in the book
- **Embedding ID**: Unique identifier for the passage in the vector store

### Agent Response
- **Content**: Generated response text
- **Source Passages**: List of passages used to generate the response
- **Status**: Success, not found, or error

## Relationships

1. A **Query** is processed by the **RAG Agent**
2. The **RAG Agent** retrieves zero or more **Retrieved Passages** based on the **Query**
3. The **RAG Agent** generates an **Agent Response** using the **Query** and **Retrieved Passages**
4. An **Agent Response** references the **Retrieved Passages** used in its generation

## Validation Rules

1. **Query** must not be empty
2. **Retrieved Passages** must have a minimum relevance score to be included
3. **Agent Response** must be grounded in the content of **Retrieved Passages**
4. If no **Retrieved Passages** meet the minimum relevance threshold, the **Agent Response** must indicate "Not found in the book"