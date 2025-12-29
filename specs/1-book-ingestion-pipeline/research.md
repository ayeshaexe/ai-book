# Research Findings: Book Website Ingestion and Embedding Pipeline

**Feature**: 1-book-ingestion-pipeline
**Created**: 2025-12-25
**Status**: Complete

## 1. Sitemap Parsing Research

### Decision: Use `lxml` for sitemap parsing
- **Rationale**: More robust XML parsing than the built-in `xml.etree.ElementTree` and better performance for larger sitemaps
- **Alternatives considered**:
  - `xml.etree.ElementTree` (Python standard library, but less robust for complex XML)
  - `beautifulsoup4` (designed for HTML, works with XML but not optimal)
  - `requests-xml` (abandoned project, not recommended)

### Technical Details
- Sitemap XML structure: `<urlset><url><loc>URL</loc></url></urlset>`
- Potential issues: Large sitemaps, nested sitemap indexes, malformed XML
- Solution: Parse with lxml, handle namespace variations, include error handling

## 2. Web Content Extraction Research

### Decision: Use `beautifulsoup4` with `requests` for HTML parsing
- **Rationale**: Most reliable for extracting clean text from HTML documents, handles malformed HTML gracefully
- **Alternatives considered**:
  - `scrapy` (overkill for this simple use case, complex setup)
  - `selenium` (unnecessary for static content, performance overhead)
  - `lxml.html` (good but more complex than needed)
  - `html2text` (loses semantic structure needed for content extraction)

### Content Extraction Strategy
- Target main content areas (likely `<main>`, `<article>`, or content-specific classes)
- Remove navigation, headers, footers, sidebars
- Preserve text structure and paragraphs
- Extract title from `<title>` tag or `<h1>`

## 3. Text Chunking Strategy Research

### Decision: Semantic chunking with sentence boundaries
- **Rationale**: Preserves context while keeping chunks within embedding model limits (typically 512-1024 tokens)
- **Alternatives considered**:
  - Fixed-length character chunks (breaks semantic meaning)
  - Paragraph-level chunks (potentially too large)
  - Token-based chunks (requires tokenization before chunking)

### Chunking Approach
- Target 800-1000 character chunks to stay well below token limits
- Respect sentence boundaries to maintain context
- Overlap chunks slightly (100-200 chars) to preserve continuity
- Handle special cases like code blocks, lists, and headers

## 4. Cohere Model Selection Research

### Decision: Use Cohere's embed-multilingual-v3.0 model
- **Rationale**:
  - Latest model with improved performance
  - Good for English content (the book's language)
  - Efficient tokenization
  - 1024-dimensional output vectors (manageable size)
- **Alternatives considered**:
  - embed-english-v3.0 (good but multilingual offers more flexibility)
  - Other providers (OpenAI, Hugging Face) (not aligned with requirements)

### Model Specifications
- Input: Text up to 512 tokens (varies by model)
- Output: 1024-dimensional vector
- Max batch size: 96 texts per request
- Rate limits: Based on tokens and requests per minute

## 5. Qdrant Collection Design Research

### Decision: Create collection optimized for book content retrieval
- **Rationale**: Qdrant Cloud Free Tier provides sufficient capacity for book content
- **Configuration**:
  - Vector size: 1024 (matching Cohere model output)
  - Distance metric: Cosine (standard for text embeddings)
  - Collection: Optimized for search rather than write-heavy operations

### Metadata Schema
- `url`: Source URL (indexed for filtering)
- `title`: Page title (indexed for filtering)
- `chunk_index`: Position within source document
- `content`: First few words of chunk (for validation)

## 6. Error Handling and Resilience Research

### Decision: Implement comprehensive error handling
- **Rationale**: Web scraping involves network requests and external APIs, both prone to failures
- **Approaches**:
  - Retry with exponential backoff
  - Fallback mechanisms for different error types
  - Logging for monitoring and debugging
  - Graceful degradation when parts of system fail

## 7. Performance Optimization Research

### Decision: Batch operations and connection pooling
- **Rationale**: Processing many pages and generating many embeddings can be time-consuming
- **Optimizations**:
  - Batch Cohere API calls (up to 96 texts)
  - Use connection pooling for HTTP requests
  - Parallel page scraping (within reasonable limits)
  - Efficient memory management for large datasets