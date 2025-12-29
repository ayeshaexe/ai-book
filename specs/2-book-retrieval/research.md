# Research: Book Data Retrieval & Pipeline Verification

## Decision: Qdrant Client Library
**Rationale**: The official qdrant-client Python library provides the most reliable and feature-complete interface to interact with Qdrant collections. It supports all necessary operations: searching, filtering, retrieving records, and accessing metadata.
**Alternatives considered**: Direct HTTP API calls using requests library - rejected because the official client provides better error handling, type safety, and maintained compatibility.

## Decision: Sitemap Parsing Approach
**Rationale**: Using the requests library to fetch the sitemap and lxml to parse XML is the most efficient and reliable approach for extracting URLs from the sitemap. The lxml library provides robust XML parsing capabilities.
**Alternatives considered**: Using BeautifulSoup for XML parsing - rejected because lxml is more efficient for XML specifically; Using feedparser library - rejected because it's primarily for RSS/Atom feeds, not sitemaps.

## Decision: Single File Architecture
**Rationale**: As specified in the requirements, all retrieval logic must reside in a single file (`backend/retrieval.py`). This simplifies deployment and maintenance while meeting the constraint.
**Alternatives considered**: Modular approach with separate files for Qdrant operations, URL validation, etc. - rejected because it violates the single file requirement.

## Decision: URL Validation Strategy
**Rationale**: Compare the set of URLs from the sitemap with the set of stored URLs in Qdrant using a simple set difference operation to identify missing URLs. This provides clear validation results.
**Alternatives considered**: Sequential validation of each URL - rejected because set operations are more efficient for bulk comparison.

## Decision: Sample Query Implementation
**Rationale**: Use Qdrant's search functionality with a sample query text and cosine similarity to validate that the embeddings are properly stored and retrievable. This confirms the retrieval mechanism works as expected.
**Alternatives considered**: Using point lookup by ID - rejected because similarity search better validates the actual retrieval use case.