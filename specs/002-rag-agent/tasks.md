---
description: "Task list for RAG agent implementation"
---

# Tasks: Agent-Based RAG Retrieval for Embedded Book Chatbot

**Input**: Design documents from `/specs/002-rag-agent/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend only**: `backend/` at repository root
- Paths shown below assume single file implementation - adjust based on plan.md structure

## Phase 1: Setup (10-12 Implementation Tasks)

**Purpose**: Implementation of the RAG agent in a single file

- [X] T001 [P] Create backend directory and requirements.txt with openai-agents, qdrant-client dependencies
- [X] T002 Create Qdrant client configuration in backend/agent.py to connect to book_embeddings collection
- [X] T003 Implement Qdrant retrieval function with search capability for book content
- [X] T004 Configure OpenAI client to use OpenRouter API with base_url=https://openrouter.ai/api/v1
- [X] T005 Create function_tool for Qdrant retrieval following OpenAI Agents SDK syntax
- [X] T006 Implement the RAG agent with instructions to answer only from retrieved book content
- [X] T007 Create main query processing function that connects user query to Qdrant retrieval
- [X] T008 Implement logic to return "Not found in the book" when no relevant content exists
- [X] T009 Integrate retrieved passages as context for the agent's response generation
- [X] T010 Test the complete RAG flow with sample queries from the book content
- [X] T011 Add environment variable handling for API keys without creating extra files
- [X] T012 Finalize agent implementation with proper error handling and validation