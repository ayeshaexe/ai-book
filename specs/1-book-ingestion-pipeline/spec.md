# Feature Specification: Book Website Ingestion and Embedding Pipeline

**Feature Branch**: `1-book-ingestion-pipeline`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Book website ingestion and embedding pipeline (Spec 1)

Target audience:
Developers implementing the ingestion layer for an embedded RAG chatbot inside a published book

Focus:
- Ingest the deployed book website
- Extract and clean book content
- Generate embeddings using **Cohere models**
- Store embeddings in **Qdrant** for downstream retrieval

Book sources:
- Deployed website URL: https://ai-book-hackathon-mu.vercel.app
- Sitemap URL: https://ai-book-hackathon-mu.vercel.app/sitemap.xml

Success criteria:
- All valid book pages are discovered via sitemap and processed
- Text content is cleaned and chunked correctly
- Embeddings are generated using Cohere without errors
- All embeddings are stored in Qdrant with correct metadata (URL, title, chunk index)
- Sample vector searches return relevant book content

Constraints:
- Scope: Ingestion and embedding only (no querying or chatbot logic)
- Backend: Python
- Data sources: Only the deployed book website
- Vector database: Qdrant Cloud (Free Tier)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ingest Book Content and Generate Embeddings (Priority: P1)

As a developer implementing an embedded RAG chatbot, I need to ingest content from a deployed book website and generate embeddings so that the chatbot can retrieve relevant book content to answer user questions.

**Why this priority**: This is the foundational functionality required for the RAG system to work. Without proper ingestion and embedding, the downstream chatbot cannot function.

**Independent Test**: The system can successfully discover all valid book pages via the sitemap, extract clean text content from each page, generate embeddings using Cohere models, and store them in Qdrant with proper metadata.

**Acceptance Scenarios**:

1. **Given** a deployed book website with a sitemap, **When** the ingestion pipeline is triggered, **Then** all valid book pages are discovered and processed
2. **Given** raw HTML content from book pages, **When** content extraction and cleaning occurs, **Then** clean text is produced with book content and irrelevant elements removed

---

### User Story 2 - Store Embeddings with Metadata (Priority: P1)

As a developer, I need to store the generated embeddings in Qdrant with proper metadata so that downstream retrieval systems can access the content with context.

**Why this priority**: Proper metadata storage is essential for accurate retrieval and attribution of the embedded content back to its source.

**Independent Test**: Embeddings are successfully stored in Qdrant with URL, title, and chunk index metadata, and can be retrieved with this information intact.

**Acceptance Scenarios**:

1. **Given** generated embeddings with associated content, **When** they are stored in Qdrant, **Then** they include URL, title, and chunk index metadata
2. **Given** embeddings stored in Qdrant, **When** they are retrieved, **Then** all metadata is preserved and accessible

---

### User Story 3 - Verify Embedding Quality (Priority: P2)

As a developer, I need to verify that the generated embeddings maintain semantic meaning so that similarity searches return relevant book content.

**Why this priority**: High-quality embeddings are essential for the RAG system to provide accurate and relevant responses to user queries.

**Independent Test**: Sample vector searches return relevant book content that matches the query intent.

**Acceptance Scenarios**:

1. **Given** a query about a specific topic from the book, **When** vector search is performed, **Then** relevant book content chunks are returned
2. **Given** embeddings generated from book content, **When** quality checks are performed, **Then** they demonstrate semantic coherence

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST discover all valid book pages by parsing the provided sitemap URL
- **FR-002**: System MUST extract clean text content from each discovered book page
- **FR-003**: System MUST chunk the extracted content into appropriately sized segments for embedding
- **FR-004**: System MUST generate embeddings using Cohere models without errors
- **FR-005**: System MUST store all generated embeddings in Qdrant with metadata (URL, title, chunk index)
- **FR-006**: System MUST handle errors gracefully during ingestion, extraction, and embedding processes
- **FR-007**: System MUST preserve the relationship between content chunks and their source pages
- **FR-008**: System MUST support the Qdrant Cloud Free Tier specifications and limitations

### Key Entities

- **BookContent**: Represents the extracted text from book pages, including the raw content and source information
- **Embedding**: Represents the vector representation of a content chunk, with associated metadata
- **SourceMetadata**: Contains information about the original source (URL, title, chunk index) for each embedding

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All valid book pages (from sitemap) are discovered and processed with 100% coverage
- **SC-002**: Text content is cleaned and chunked with 95% accuracy (meaning meaningful content preserved, irrelevant elements removed)
- **SC-003**: Embeddings are generated successfully with 99% success rate (less than 1% failure rate)
- **SC-004**: All embeddings are stored in Qdrant with correct metadata, with 99% accuracy
- **SC-005**: Sample vector searches return relevant book content with 90% relevance (measured by manual review of search results)
- **SC-006**: The entire ingestion pipeline completes within a reasonable timeframe (less than 1 hour for a typical book website)