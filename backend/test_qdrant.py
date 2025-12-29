"""
Test script to check Qdrant connection and data
"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_api_key:
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key
    )
else:
    qdrant_client = QdrantClient(url=qdrant_url)

# Set collection name from environment or default
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

print(f"Connecting to Qdrant at: {qdrant_url}")
print(f"Using collection: {collection_name}")

try:
    # Check if collection exists and get its info
    collection_info = qdrant_client.get_collection(collection_name)
    print(f"Collection exists. Points count: {collection_info.points_count}")
    print(f"Collection vectors count: {collection_info.vectors_count}")

    # Try to get a sample of points to see if there's data
    if collection_info.points_count > 0:
        # Get first 5 points as a sample
        records, _ = qdrant_client.scroll(
            collection_name=collection_name,
            limit=5
        )

        print("\nSample records from the collection:")
        for i, record in enumerate(records):
            print(f"Record {i+1}:")
            print(f"  ID: {record.id}")
            print(f"  Content preview: {str(record.payload.get('content_preview', ''))[:200]}...")
            print(f"  Title: {record.payload.get('title', 'No title')}")
            print(f"  URL: {record.payload.get('url', 'No URL')}")
            print(f"  Payload keys: {list(record.payload.keys())}")
            print()
    else:
        print("Collection is empty - no points found.")

except Exception as e:
    print(f"Error accessing collection: {e}")