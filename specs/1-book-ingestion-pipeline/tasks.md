# Implementation Tasks: Book Website Ingestion and Embedding Pipeline

**Feature**: 1-book-ingestion-pipeline
**Created**: 2025-12-25
**Status**: Draft

## Phase 1: Setup
**Goal**: Initialize project structure and dependencies

- [X] T001 Create backend/ directory structure for the ingestion pipeline
- [X] T002 Set up Python project using uv with required dependencies (requests, beautifulsoup4, lxml, cohere, qdrant-client, python-dotenv)
- [X] T003 Create environment configuration file (.env) with placeholders for API keys and URLs

## Phase 2: Foundational Components
**Goal**: Implement core utilities needed by all user stories

- [X] T004 Implement sitemap parser to fetch and parse https://ai-book-hackathon-mu.vercel.app/sitemap.xml
- [X] T005 Create web content extractor using requests and beautifulsoup4 to extract clean text from book pages
- [X] T006 Build text chunking utility that splits content into semantically meaningful segments

## Phase 3: User Story 1 - Ingest Book Content and Generate Embeddings (P1)
**Goal**: Core functionality to discover, extract, and embed book content

**Independent Test Criteria**: The system can successfully discover all valid book pages via the sitemap, extract clean text content from each page, generate embeddings using Cohere models, and store them in Qdrant with proper metadata.

- [X] T007 [US1] Implement main ingestion function that orchestrates sitemap discovery and page processing
- [X] T008 [US1] Integrate Cohere API to generate embeddings using embed-multilingual-v3.0 model
- [X] T009 [US1] Add error handling and retry logic for network requests and API calls

## Phase 4: User Story 2 - Store Embeddings with Metadata (P1)
**Goal**: Store generated embeddings in Qdrant with proper metadata for retrieval

**Independent Test Criteria**: Embeddings are successfully stored in Qdrant with URL, title, and chunk index metadata, and can be retrieved with this information intact.

- [X] T010 [US2] Set up Qdrant collection for storing book embeddings with appropriate vector size (1024) and metadata fields
- [X] T011 [US2] Implement storage function to save embeddings with metadata (URL, title, chunk_index) to Qdrant

## Dependencies
- US1 (User Story 1) must be completed before US2 can be fully tested
- Foundational components (Phase 2) must be completed before user stories can be implemented

## Parallel Execution Opportunities
- T001, T002, T003 (setup phase) can be executed in parallel
- T004, T005, T006 (foundational components) can be developed in parallel
- T007, T008, T009 (US1) can be developed in parallel after foundational components are complete

## Implementation Strategy
- **MVP Scope**: Implement US1 (sitemap parsing, content extraction, embedding generation) with basic Qdrant storage
- **Incremental Delivery**: First deliver the ability to ingest and embed a single page, then scale to full sitemap processing
- **Quality Focus**: Prioritize accuracy of content extraction and embedding generation over speed initially