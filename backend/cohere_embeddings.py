import os
import cohere
import logging
from typing import List, Dict
from dotenv import load_dotenv
import time

# Load environment variables
load_dotenv()

# Initialize Cohere client
cohere_api_key = os.getenv('COHERE_API_KEY')
if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")

client = cohere.Client(cohere_api_key)

logger = logging.getLogger(__name__)

def generate_embeddings(chunks: List[Dict], model: str = 'embed-multilingual-v3.0') -> List[Dict]:
    """
    Generate embeddings for content chunks using Cohere API.

    Args:
        chunks (List[Dict]): List of content chunks with metadata
        model (str): Cohere embedding model to use

    Returns:
        List[Dict]: List of chunks with embeddings and metadata
    """
    if not chunks:
        logger.info("No chunks provided for embedding generation")
        return []

    # Prepare text for embedding
    texts = [chunk['content'] for chunk in chunks]

    all_embeddings_data = []
    batch_size = 96  # Cohere's recommended batch size

    # Process in batches to respect API limits
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i + batch_size]
        batch_chunks = chunks[i:i + batch_size]

        try:
            # Generate embeddings
            response = client.embed(
                texts=batch_texts,
                model=model,
                input_type="search_document"  # Appropriate for document search
            )

            # Attach embeddings to the corresponding chunks
            for j, embedding in enumerate(response.embeddings):
                chunk_data = batch_chunks[j].copy()
                chunk_data['embedding'] = embedding
                chunk_data['model'] = model
                all_embeddings_data.append(chunk_data)

            logger.info(f"Generated embeddings for batch {i//batch_size + 1}/{(len(texts) - 1)//batch_size + 1}")

        except Exception as e:
            logger.error(f"Error generating embeddings for batch {i//batch_size + 1}: {e}")
            # Add error info to chunks and continue
            for chunk in batch_chunks:
                chunk_data = chunk.copy()
                chunk_data['embedding'] = None
                chunk_data['error'] = str(e)
                all_embeddings_data.append(chunk_data)

        # Add a small delay to avoid rate limiting
        time.sleep(0.1)

    logger.info(f"Successfully generated embeddings for {len(all_embeddings_data)} chunks")
    return all_embeddings_data

def generate_single_embedding(text: str, model: str = 'embed-multilingual-v3.0') -> List[float]:
    """
    Generate a single embedding for a text string.

    Args:
        text (str): Text to embed
        model (str): Cohere embedding model to use

    Returns:
        List[float]: The embedding vector
    """
    try:
        response = client.embed(
            texts=[text],
            model=model,
            input_type="search_query"  # Appropriate for search queries
        )
        return response.embeddings[0]
    except Exception as e:
        logger.error(f"Error generating single embedding: {e}")
        raise

def validate_embeddings(embeddings_data: List[Dict]) -> bool:
    """
    Validate that embeddings were generated successfully.

    Args:
        embeddings_data (List[Dict]): List of chunks with embeddings

    Returns:
        bool: True if all embeddings are valid, False otherwise
    """
    if not embeddings_data:
        logger.warning("No embeddings data to validate")
        return False

    valid_count = 0
    for item in embeddings_data:
        if item.get('embedding') is not None and isinstance(item['embedding'], list):
            valid_count += 1
        elif item.get('error'):
            logger.error(f"Embedding generation failed for chunk: {item.get('error')}")

    success_rate = valid_count / len(embeddings_data)
    logger.info(f"Embedding validation: {valid_count}/{len(embeddings_data)} ({success_rate:.2%}) successful")

    return success_rate >= 0.95  # Consider successful if 95% or more are valid

if __name__ == "__main__":
    # Example usage
    sample_chunks = [
        {
            'url': 'https://example.com/page1',
            'title': 'Sample Page 1',
            'content': 'This is the content of the first sample page.',
            'chunk_index': 0,
            'length': 42
        },
        {
            'url': 'https://example.com/page2',
            'title': 'Sample Page 2',
            'content': 'This is the content of the second sample page.',
            'chunk_index': 0,
            'length': 42
        }
    ]

    try:
        embeddings_data = generate_embeddings(sample_chunks)
        print(f"Generated embeddings for {len(embeddings_data)} chunks")
        if embeddings_data and 'embedding' in embeddings_data[0]:
            print(f"Embedding dimension: {len(embeddings_data[0]['embedding'])}")
    except Exception as e:
        print(f"Error: {e}")