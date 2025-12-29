#!/usr/bin/env python3
"""
Book Website Ingestion and Embedding Pipeline

This script implements the complete pipeline for:
1. Fetching and parsing the sitemap
2. Crawling each page and extracting content
3. Chunking text into meaningful segments
4. Generating embeddings using Cohere
5. Storing embeddings with metadata in Qdrant
6. Verifying the pipeline with sample queries
"""

import os
import sys
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import our modules
try:
    from ingestion_pipeline import run_ingestion_pipeline
    from cohere_embeddings import generate_single_embedding
    from qdrant_storage import search_similar, verify_storage
except ImportError as e:
    logger.error(f"Failed to import required modules: {e}")
    sys.exit(1)

def run_verification_test():
    """
    Run verification tests to ensure the pipeline works correctly.
    """
    logger.info("Running verification tests...")

    # Verify storage
    if verify_storage():
        logger.info("✓ Storage verification passed")
    else:
        logger.error("✗ Storage verification failed")
        return False

    # Test search functionality with a sample query
    try:
        sample_query = "What is the main topic of the book?"
        query_embedding = generate_single_embedding(sample_query, model='embed-multilingual-v3.0')

        results = search_similar(query_embedding, top_k=3)

        if results:
            logger.info(f"✓ Search test passed - found {len(results)} relevant results")
            for i, result in enumerate(results[:2]):  # Show first 2 results
                logger.info(f"  Result {i+1}: Score {result['score']:.3f} - {result['payload']['title'][:50]}...")
        else:
            logger.warning("⚠ Search test returned no results (this may be expected if no content is stored yet)")

    except Exception as e:
        logger.error(f"✗ Search test failed: {e}")
        return False

    return True

def main():
    """
    Main function to run the complete ingestion pipeline.
    """
    logger.info("Starting Book Website Ingestion and Embedding Pipeline")

    try:
        # Run the ingestion pipeline
        logger.info("Step 1: Running ingestion pipeline...")
        success = run_ingestion_pipeline()

        if not success:
            logger.error("Ingestion pipeline failed")
            return 1

        logger.info("✓ Ingestion pipeline completed successfully")

        # Run verification tests
        logger.info("Step 2: Running verification tests...")
        verification_success = run_verification_test()

        if not verification_success:
            logger.error("Verification tests failed")
            return 1

        logger.info("✓ All verification tests passed")

        logger.info("🎉 Book ingestion pipeline completed successfully!")
        return 0

    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Pipeline failed with error: {e}")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
