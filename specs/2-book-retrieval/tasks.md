# Tasks: Book Data Retrieval & Pipeline Verification

**Feature**: Book Data Retrieval & Pipeline Verification
**Branch**: `2-book-retrieval`
**Generated**: 2025-12-25
**Input**: specs/2-book-retrieval/spec.md, specs/2-book-retrieval/plan.md

## Overview

Implementation of book content retrieval and verification functionality in `backend/retrieval.py`. The solution will connect to Qdrant, validate stored URLs match the sitemap, verify metadata completeness, and execute sample retrieval queries.

## Dependencies

- User Story 2 (URL validation) depends on User Story 1 (basic retrieval) being complete
- User Story 3 (sample queries) depends on User Story 1 (basic retrieval) being complete

## Parallel Execution Examples

- [P] T002, T003 can be executed in parallel after T001
- [P] US2 and US3 tasks can be developed in parallel after US1 foundation is complete

## Implementation Strategy

1. **MVP**: Complete User Story 1 (basic retrieval) to establish core Qdrant connectivity
2. **Increment 1**: Add URL validation (User Story 2) and sample queries (User Story 3)
3. **Polish**: Error handling and main execution function

---

## Phase 1: Setup

- [X] T001 Create initial backend/retrieval.py file with imports
- [X] T002 Install required dependencies (qdrant-client, requests, lxml)

## Phase 2: Foundational Components

- [X] T003 [P] Implement Qdrant client initialization function
- [X] T004 [P] Create function to retrieve all book content chunks from Qdrant
- [X] T005 [P] Create function to validate required metadata in chunks

## Phase 3: [US1] Verify Book Content Retrieval

**Goal**: Retrieve book content from Qdrant to verify ingestion pipeline worked correctly

**Independent Test**: Running the retrieval script successfully fetches content from the Qdrant collection, proving the ingestion pipeline stored data correctly.

**Acceptance**:
- Script returns all stored text chunks with their metadata
- Each chunk includes `url`, `chunk_index`, and `content_preview` metadata

- [X] T006 [US1] Implement main retrieval function that fetches and validates content

## Phase 4: [US2] Validate URL Presence in Database

**Goal**: Verify that all expected book URLs exist in Qdrant to confirm ingestion completeness

**Independent Test**: Running URL validation checks against Qdrant collection confirms all expected URLs are present.

**Acceptance**:
- Sitemap URL https://ai-book-hackathon-mu.vercel.app/sitemap.xml is present in collection
- All expected book page URLs are present

- [X] T007 [US2] Implement function to fetch and parse sitemap XML
- [X] T008 [US2] Create function to compare sitemap URLs with stored URLs

## Phase 5: [US3] Execute Sample Queries for Retrieval Testing

**Goal**: Run sample similarity queries to verify retrieval functionality works properly

**Independent Test**: Executing sample queries against Qdrant collection verifies relevant results are returned.

**Acceptance**:
- Similarity queries return relevant book chunks based on semantic similarity
- Results include proper metadata (`url`, `chunk_index`, `text`)

- [X] T009 [US3] Implement function to run sample similarity queries
- [X] T010 [US3] Add function to format and display query results with metadata

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T011 Add error handling for Qdrant connection failures
- [X] T012 Create main execution function that runs all verification steps