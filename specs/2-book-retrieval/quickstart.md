# Quickstart: Book Data Retrieval & Pipeline Verification

## Prerequisites

- Python 3.11+
- Qdrant instance with `book_embeddings` collection populated
- Access to sitemap at https://ai-book-hackathon-mu.vercel.app/sitemap.xml

## Setup

1. Install dependencies:
   ```bash
   pip install qdrant-client requests lxml
   ```

2. Ensure Qdrant is accessible and the `book_embeddings` collection exists

3. Verify the sitemap URL is accessible

## Running the Retrieval Script

1. Execute the retrieval script:
   ```bash
   cd backend
   python retrieval.py
   ```

2. The script will:
   - Fetch and parse the sitemap
   - Connect to Qdrant and retrieve stored URLs
   - Compare sitemap URLs with stored URLs
   - Run sample similarity queries
   - Output validation results

## Expected Output

The script will display:
- Number of URLs in sitemap vs. stored in Qdrant
- List of any missing URLs
- Sample retrieval query results
- Metadata validation results