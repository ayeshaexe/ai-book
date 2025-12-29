# Implementation Plan: Book Website Ingestion and Embedding Pipeline

**Feature**: 1-book-ingestion-pipeline
**Created**: 2025-12-25
**Status**: Draft
**Author**: Claude

## Technical Context

The system needs to ingest content from a deployed book website (https://ai-book-hackathon-mu.vercel.app), extract and clean the text content, generate embeddings using Cohere models, and store them in Qdrant with proper metadata. This forms the ingestion layer for an embedded RAG chatbot inside a published book.

### Architecture Overview
- **Backend**: Python-based ingestion pipeline
- **Data Source**: Book website with sitemap.xml
- **Embedding Service**: Cohere API
- **Vector Database**: Qdrant Cloud (Free Tier)
- **File Structure**: backend/ folder with main.py implementation

### Technology Stack
- Python 3.9+
- `uv` for project management
- Requests/BeautifulSoup for web crawling
- Cohere Python SDK for embeddings
- Qdrant Python client for vector storage
- lxml for sitemap parsing

### Key Components
1. Sitemap parser to discover all book pages
2. Web crawler to extract content from each page
3. Text cleaner and chunker for semantic processing
4. Cohere embedding generator
5. Qdrant storage with metadata
6. Sample query verification

## Constitution Check

### Alignment with Constitution Principles
- **Reproducibility**: Code will be executable and tested with clear examples
- **Accuracy**: Implementation will follow best practices for web scraping and embedding generation
- **Clarity**: Code will be well-documented with clear variable names and comments
- **Consistency**: Follows standard Python conventions and project structure

### Potential Violations
- None identified - implementation aligns with constitution principles

## Gates

### Gate 1: Technical Feasibility ✅
- Python web scraping and embedding generation are proven technologies
- Cohere API and Qdrant integration are well-documented
- Sitemap parsing is a standard practice

### Gate 2: Resource Constraints ✅
- Qdrant Cloud Free Tier supports the required functionality
- Cohere API offers sufficient rate limits for ingestion
- Implementation can be completed within reasonable timeframes

### Gate 3: Architecture Alignment ✅
- Solution aligns with RAG chatbot architecture mentioned in constitution
- Uses approved technologies (Cohere, Qdrant)
- Follows backend service pattern

## Phase 0: Research & Analysis

### Research Tasks Completed

#### 1. Sitemap Parsing
- **Decision**: Use `xml.etree.ElementTree` or `lxml` to parse sitemap.xml
- **Rationale**: Both are reliable for parsing XML sitemaps; `lxml` has better performance
- **Alternatives considered**: `beautifulsoup4`, custom regex (rejected for fragility)

#### 2. Web Content Extraction
- **Decision**: Use `beautifulsoup4` with `requests` for HTML parsing
- **Rationale**: Most reliable for extracting clean text from HTML documents
- **Alternatives considered**: `scrapy`, `selenium` (overkill for static content)

#### 3. Text Chunking Strategy
- **Decision**: Semantic chunking based on document structure and sentence boundaries
- **Rationale**: Preserves context while keeping chunks within embedding model limits
- **Alternatives considered**: Fixed-length character chunks (loses semantic meaning)

#### 4. Cohere Model Selection
- **Decision**: Use Cohere's embed-multilingual-v3.0 model for text embeddings
- **Rationale**: Good performance for English content with efficient tokenization
- **Alternatives considered**: embed-english-v3.0, other embedding providers

#### 5. Qdrant Collection Design
- **Decision**: Create collection with appropriate vector size and metadata fields
- **Rationale**: Qdrant Cloud Free Tier supports up to 5 collections with 10k vectors each
- **Alternatives considered**: Different vector databases (sticking with approved tech)

## Phase 1: Design & Contracts

### Data Model

#### BookContent Entity
- **id**: Unique identifier for the content chunk (UUID)
- **url**: Source URL of the original page
- **title**: Page title extracted from HTML
- **content**: Clean text content of the chunk
- **chunk_index**: Sequential index for content chunks from the same page
- **created_at**: Timestamp of when the content was ingested

#### Embedding Entity
- **vector_id**: Unique identifier for the vector in Qdrant
- **content_id**: Reference to BookContent.id
- **vector**: The embedding vector (1024-dimensional for Cohere multilingual model)
- **metadata**: JSON object containing {url, title, chunk_index}
- **created_at**: Timestamp of when the embedding was generated

#### SourceMetadata Entity
- **url**: The original page URL
- **title**: The page title
- **chunk_index**: Index of the content chunk in the original page
- **ingestion_timestamp**: When the page was processed

### API Contracts

Since this is an ingestion pipeline rather than an API service, the main "contract" is the internal structure of the main function and its components:

#### Sitemap Parser Interface
```
parse_sitemap(sitemap_url: str) -> List[str]
```
- Input: URL to sitemap.xml
- Output: List of page URLs to process

#### Content Extractor Interface
```
extract_content(page_url: str) -> Tuple[str, str]
```
- Input: URL of a book page
- Output: (page_title, clean_content_text)

#### Text Chunker Interface
```
chunk_text(content: str, max_chunk_size: int = 1000) -> List[Dict]
```
- Input: Clean content text and maximum chunk size
- Output: List of chunks with metadata

#### Embedding Generator Interface
```
generate_embeddings(chunks: List[Dict]) -> List[Dict]
```
- Input: List of content chunks
- Output: List of embeddings with metadata

#### Qdrant Storage Interface
```
store_embeddings(embeddings: List[Dict]) -> bool
```
- Input: List of embedding objects with metadata
- Output: Success status

### Quickstart Guide

#### Setup Instructions
1. Install `uv` package manager
2. Create `backend/` directory
3. Initialize project with `uv init`
4. Install dependencies: `uv pip install requests beautifulsoup4 lxml cohere qdrant-client python-dotenv`
5. Set up environment variables for Cohere and Qdrant
6. Run the ingestion script: `python main.py`

#### Prerequisites
- Python 3.9+
- `uv` package manager
- Cohere API key
- Qdrant Cloud cluster URL and API key

#### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `BOOK_SITEMAP_URL`: URL to the book sitemap (default: https://ai-book-hackathon-mu.vercel.app/sitemap.xml)

## Phase 2: Implementation Strategy

### Development Approach
1. Create project structure in `backend/` directory
2. Implement sitemap parsing functionality
3. Build web content extraction and cleaning
4. Develop text chunking logic
5. Integrate Cohere embedding generation
6. Implement Qdrant storage
7. Add sample query verification
8. Add error handling and logging
9. Test with the target book website

### Risk Mitigation
- **Rate Limiting**: Implement exponential backoff for API calls
- **Network Errors**: Add retry logic for web requests
- **Embedding Failures**: Log failed embeddings for reprocessing
- **Qdrant Issues**: Implement proper error handling and validation

### Success Criteria Validation
- All pages from sitemap are processed (SC-001)
- Content is cleaned with 95% accuracy (SC-002)
- Embeddings generated with 99% success rate (SC-003)
- Metadata stored correctly with 99% accuracy (SC-004)
- Sample queries return relevant results (SC-005)
- Pipeline completes in under 1 hour (SC-006)

## Post-Implementation Verification

### Testing Strategy
1. Unit tests for each component
2. Integration tests for end-to-end pipeline
3. Manual verification of sample queries
4. Performance testing with full book dataset

### Quality Assurance
- Code review checklist
- Embedding quality validation
- Metadata integrity verification
- Performance benchmarking