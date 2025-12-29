# Research: Agent-Based RAG Retrieval for Embedded Book Chatbot

**Feature**: Agent-Based RAG Retrieval for Embedded Book Chatbot
**Date**: 2025-12-26
**Branch**: 002-rag-agent

## Research Summary

This research document addresses the technical requirements for implementing a RAG agent that uses OpenAI Agents SDK with OpenRouter API to retrieve information from a Qdrant vector database.

## Decision: Qdrant Integration with OpenAI Agents

**Rationale**: The RAG agent needs to retrieve relevant passages from the existing Qdrant collection `book_embeddings` and pass them to the OpenAI Agent for response generation. This requires creating a custom function tool that connects to Qdrant, performs semantic search, and returns relevant passages.

**Alternatives considered**:
1. Using OpenAI's built-in FileSearchTool - Not suitable as it doesn't connect to our existing Qdrant collection
2. Building a custom function tool - Most appropriate as it allows direct access to our Qdrant collection
3. Using LangChain integration - Would add unnecessary complexity for this simple use case

## Decision: OpenRouter API Configuration

**Rationale**: The agent needs to use OpenRouter API instead of OpenAI API. This requires configuring the OpenAI client with the OpenRouter base URL and API key.

**Implementation approach**:
- Set `base_url="https://openrouter.ai/api/v1"` when initializing the OpenAI client
- Use `OPENROUTER_API_KEY` environment variable for authentication
- Ensure the model specified is available on OpenRouter

## Decision: RAG Agent Architecture

**Rationale**: The agent architecture needs to follow the flow: Query → Retrieve from Qdrant → Generate response with retrieved context

**Implementation approach**:
1. Create a Qdrant retrieval tool that searches the `book_embeddings` collection
2. Create an agent with instructions to use retrieved passages as context
3. Implement fallback response when no relevant content is found

## Decision: Qdrant Client Configuration

**Rationale**: Need to connect to the existing Qdrant collection `book_embeddings` for retrieval.

**Implementation approach**:
- Use qdrant-client library to connect to Qdrant
- Configure collection name as `book_embeddings`
- Use environment variables for Qdrant connection parameters (URL, API key if needed)

## Decision: Agent Response Handling

**Rationale**: The agent must return "Not found in the book" when no relevant content exists in the book for a given query.

**Implementation approach**:
- The retrieval tool will return an empty result or a specific indicator when no relevant passages are found
- The agent instructions will specify how to handle this case