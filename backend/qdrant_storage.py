import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from dotenv import load_dotenv
import logging
import uuid

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

# Initialize Qdrant client - First try environment variables for cloud, then fall back to local
qdrant_url = os.getenv('QDRANT_URL')
qdrant_api_key = os.getenv('QDRANT_API_KEY')

if qdrant_url and qdrant_api_key:
    # Use cloud Qdrant instance
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        https=True
    )
    print(f"Using Qdrant Cloud instance: {qdrant_url}")
else:
    # Use local in-memory Qdrant instance
    from qdrant_client import QdrantClient
    client = QdrantClient(":memory:")  # In-memory instance for local development
    print("Using local in-memory Qdrant instance")

def setup_qdrant_collection(collection_name: str = "book_embeddings", vector_size: int = 1024) -> bool:
    """
    Set up Qdrant collection for storing book embeddings.

    Args:
        collection_name (str): Name of the collection to create
        vector_size (int): Size of the embedding vectors (default 1024 for Cohere multilingual model)

    Returns:
        bool: True if collection exists or was created successfully, False otherwise
    """
    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        if collection_name in collection_names:
            logger.info(f"Collection '{collection_name}' already exists")
            return True

        # Create collection with appropriate vector size and metadata
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
        )

        # Create payload index for URL for faster filtering
        client.create_payload_index(
            collection_name=collection_name,
            field_name="url",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        # Create payload index for title for faster filtering
        client.create_payload_index(
            collection_name=collection_name,
            field_name="title",
            field_schema=models.PayloadSchemaType.TEXT
        )

        logger.info(f"Created collection '{collection_name}' with vector size {vector_size}")
        return True

    except Exception as e:
        logger.error(f"Error setting up Qdrant collection: {e}")
        return False

def store_embeddings(embeddings_data: List[Dict], collection_name: str = "book_embeddings") -> bool:
    """
    Store embeddings with metadata to Qdrant.

    Args:
        embeddings_data (List[Dict]): List of chunks with embeddings and metadata
        collection_name (str): Name of the Qdrant collection to store embeddings

    Returns:
        bool: True if storage was successful, False otherwise
    """
    if not embeddings_data:
        logger.warning("No embeddings data provided for storage")
        return True  # Consider this successful if no data to store

    try:
        # Ensure collection exists
        if not setup_qdrant_collection(collection_name):
            logger.error(f"Failed to set up collection '{collection_name}'")
            return False

        # Prepare points for insertion
        points = []
        for item in embeddings_data:
            if item.get('embedding') is None:
                logger.warning(f"Skipping item with no embedding: {item.get('url', 'Unknown URL')}")
                continue

            # Create a unique ID for the point
            point_id = str(uuid.uuid4())

            # Prepare the payload with metadata
            payload = {
                "url": item.get('url', ''),
                "title": item.get('title', ''),
                "chunk_index": item.get('chunk_index', 0),
                "content_preview": item.get('content', '')[:200],  # Store first 200 chars as preview
                "length": item.get('length', 0),
                "model": item.get('model', 'unknown')
            }

            # Create the point
            point = models.PointStruct(
                id=point_id,
                vector=item['embedding'],
                payload=payload
            )
            points.append(point)

        if not points:
            logger.warning("No valid points to store after filtering out invalid embeddings")
            return True

        # Upload points to Qdrant
        client.upload_points(
            collection_name=collection_name,
            points=points,
            wait=True  # Wait for the operation to complete
        )

        logger.info(f"Successfully stored {len(points)} embeddings in collection '{collection_name}'")
        return True

    except Exception as e:
        logger.error(f"Error storing embeddings in Qdrant: {e}")
        return False

def search_similar(query_embedding: List[float], collection_name: str = "book_embeddings",
                  top_k: int = 5) -> List[Dict]:
    """
    Search for similar embeddings in Qdrant.

    Args:
        query_embedding (List[float]): The embedding vector to search for
        collection_name (str): Name of the Qdrant collection to search
        top_k (int): Number of similar results to return

    Returns:
        List[Dict]: List of similar embeddings with metadata
    """
    try:
        # Perform the search
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "score": result.score,
                "payload": result.payload,
                "content_preview": result.payload.get("content_preview", "")
            })

        logger.info(f"Found {len(results)} similar embeddings")
        return results

    except Exception as e:
        logger.error(f"Error searching for similar embeddings: {e}")
        return []

def verify_storage(collection_name: str = "book_embeddings") -> bool:
    """
    Verify that embeddings were stored correctly in Qdrant.

    Args:
        collection_name (str): Name of the collection to verify

    Returns:
        bool: True if storage verification passed, False otherwise
    """
    try:
        # Get collection info
        collection_info = client.get_collection(collection_name)
        count = collection_info.points_count

        logger.info(f"Collection '{collection_name}' has {count} points")

        if count > 0:
            # Sample a point to verify metadata
            sample_points = client.scroll(
                collection_name=collection_name,
                limit=1,
                with_payload=True
            )

            if sample_points[0]:
                sample_payload = sample_points[0][0].payload
                required_fields = ["url", "title", "chunk_index"]

                for field in required_fields:
                    if field not in sample_payload:
                        logger.error(f"Missing required field '{field}' in payload: {sample_payload}")
                        return False

                logger.info(f"Storage verification passed - found required fields in sample payload")
                return True
            else:
                logger.error("No points found in collection")
                return False
        else:
            logger.warning("Collection is empty")
            return False

    except Exception as e:
        logger.error(f"Error verifying storage: {e}")
        return False

if __name__ == "__main__":
    # Example usage
    # This would require valid embeddings to test
    print("Qdrant storage module ready")
    print(f"Qdrant URL: {qdrant_url}")
    print(f"Collection: book_embeddings")