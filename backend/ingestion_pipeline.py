import os
import logging
from typing import List, Dict
from dotenv import load_dotenv
from sitemap_parser import fetch_and_parse_sitemap
from content_extractor import extract_content
from text_chunker import chunk_text
from cohere_embeddings import generate_embeddings
from qdrant_storage import store_embeddings

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def process_page(page_url: str) -> List[Dict]:
    """
    Process a single book page: extract content, chunk it, and prepare for embedding.

    Args:
        page_url (str): URL of the book page to process

    Returns:
        List[Dict]: List of content chunks with metadata
    """
    try:
        # Extract content from the page
        title, content = extract_content(page_url)

        # Chunk the content
        chunks = chunk_text(content)

        # Prepare chunks with metadata
        processed_chunks = []
        for chunk in chunks:
            processed_chunks.append({
                'url': page_url,
                'title': title,
                'content': chunk['content'],
                'chunk_index': chunk['chunk_index'],
                'length': chunk['length']
            })

        logger.info(f"Processed page {page_url}: {len(processed_chunks)} chunks")
        return processed_chunks

    except Exception as e:
        logger.error(f"Error processing page {page_url}: {e}")
        return []

def run_ingestion_pipeline(sitemap_url: str = None) -> bool:
    """
    Main ingestion pipeline that orchestrates sitemap discovery and page processing.

    Args:
        sitemap_url (str, optional): URL to sitemap.xml. If None, uses environment variable.

    Returns:
        bool: True if pipeline completed successfully, False otherwise
    """
    try:
        # Use provided sitemap URL or environment variable
        if sitemap_url is None:
            sitemap_url = os.getenv('BOOK_SITEMAP_URL', 'https://ai-book-hackathon-mu.vercel.app/sitemap.xml')

        logger.info(f"Starting ingestion pipeline for sitemap: {sitemap_url}")

        # Fetch and parse sitemap
        urls = fetch_and_parse_sitemap(sitemap_url)
        logger.info(f"Discovered {len(urls)} URLs to process")

        all_chunks = []

        # Process each URL
        for i, url in enumerate(urls):
            logger.info(f"Processing {i+1}/{len(urls)}: {url}")
            try:
                chunks = process_page(url)
                all_chunks.extend(chunks)
                logger.info(f"Completed processing {url}")
            except Exception as e:
                logger.error(f"Failed to process {url}: {e}")
                continue

        logger.info(f"Completed processing all pages. Total chunks: {len(all_chunks)}")

        # Generate embeddings for all chunks
        logger.info("Generating embeddings for all chunks...")
        embeddings_data = generate_embeddings(all_chunks)
        logger.info(f"Generated embeddings for {len(embeddings_data)} chunks")

        # Store embeddings in Qdrant
        logger.info("Storing embeddings in Qdrant...")
        success = store_embeddings(embeddings_data)

        if success:
            logger.info("Ingestion pipeline completed successfully!")
            return True
        else:
            logger.error("Failed to store embeddings in Qdrant")
            return False

    except Exception as e:
        logger.error(f"Error in ingestion pipeline: {e}")
        return False

def process_single_page(page_url: str) -> bool:
    """
    Process a single page without running the full pipeline.

    Args:
        page_url (str): URL of the page to process

    Returns:
        bool: True if processing completed successfully, False otherwise
    """
    try:
        logger.info(f"Processing single page: {page_url}")
        chunks = process_page(page_url)

        if not chunks:
            logger.error(f"No chunks generated for {page_url}")
            return False

        # Generate embeddings for chunks
        embeddings_data = generate_embeddings(chunks)

        # Store embeddings in Qdrant
        success = store_embeddings(embeddings_data)

        if success:
            logger.info(f"Successfully processed single page: {page_url}")
            return True
        else:
            logger.error(f"Failed to store embeddings for {page_url}")
            return False

    except Exception as e:
        logger.error(f"Error processing single page {page_url}: {e}")
        return False

if __name__ == "__main__":
    # Run the full ingestion pipeline
    success = run_ingestion_pipeline()
    if success:
        print("Ingestion pipeline completed successfully!")
    else:
        print("Ingestion pipeline failed!")