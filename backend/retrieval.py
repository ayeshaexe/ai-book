"""
Book Data Retrieval & Pipeline Verification

This module provides functionality to retrieve and verify book content from Qdrant,
validate stored URLs match the sitemap, verify metadata completeness, and execute
sample retrieval queries.
"""
import os
import requests
from typing import List, Dict, Set, Optional
from lxml import etree
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


def initialize_qdrant_client() -> QdrantClient:
    """
    Initialize and return a Qdrant client instance.

    Returns:
        QdrantClient: Configured Qdrant client instance

    Raises:
        ConnectionError: If unable to connect to Qdrant
        ValueError: If required environment variables are missing
    """
    # Use environment variables for configuration, with defaults
    host = os.getenv('QDRANT_HOST', 'localhost')
    port = int(os.getenv('QDRANT_PORT', 6333))
    https = os.getenv('QDRANT_HTTPS', 'false').lower() == 'true'

    try:
        # If using a cloud instance
        if os.getenv('QDRANT_API_KEY') and os.getenv('QDRANT_URL'):
            client = QdrantClient(
                url=os.getenv('QDRANT_URL'),
                api_key=os.getenv('QDRANT_API_KEY')
            )
        else:
            # Local instance
            client = QdrantClient(
                host=host,
                port=port,
                https=https
            )

        # Test the connection
        client.get_collections()

        return client
    except Exception as e:
        raise ConnectionError(f"Failed to connect to Qdrant: {str(e)}")


def retrieve_all_book_chunks(collection_name: str = "book_embeddings") -> List[Dict]:
    """
    Retrieve all book content chunks from Qdrant collection.

    Args:
        collection_name (str): Name of the Qdrant collection to query

    Returns:
        List[Dict]: List of all stored book content chunks with their metadata

    Raises:
        ConnectionError: If unable to connect to Qdrant
        Exception: If collection doesn't exist or other Qdrant errors occur
    """
    client = initialize_qdrant_client()

    try:
        # Check if collection exists
        client.get_collection(collection_name)
    except Exception as e:
        raise Exception(f"Collection '{collection_name}' does not exist or is not accessible: {str(e)}")

    chunks = []
    scroll_id = None

    try:
        while True:
            # Get a batch of points from the collection
            records, scroll_id = client.scroll(
                collection_name=collection_name,
                limit=10000,  # Adjust as needed
                with_payload=True,
                with_vectors=False,
                offset=scroll_id
            )

            for record in records:
                chunk = {
                    'id': record.id,
                    'payload': record.payload
                }
                chunks.append(chunk)

            if scroll_id is None:
                break

    except Exception as e:
        raise Exception(f"Error retrieving chunks from Qdrant: {str(e)}")

    return chunks


def validate_metadata_completeness(chunks: List[Dict]) -> Dict[str, any]:
    """
    Validate that required metadata is present in all chunks.

    Args:
        chunks (List[Dict]): List of book content chunks to validate

    Returns:
        Dict: Validation results including counts and missing fields
    """
    required_fields = {'url', 'chunk_index', 'content_preview'}  # Using content_preview instead of text
    total_chunks = len(chunks)
    valid_chunks = 0
    missing_fields = []

    for chunk in chunks:
        payload = chunk.get('payload', {})
        payload_keys = set(payload.keys())

        missing = required_fields - payload_keys
        if not missing:
            valid_chunks += 1
        else:
            missing_fields.append({
                'id': chunk.get('id'),
                'missing': list(missing)
            })

    return {
        'total_chunks': total_chunks,
        'valid_chunks': valid_chunks,
        'invalid_chunks': total_chunks - valid_chunks,
        'missing_fields': missing_fields
    }


def fetch_and_parse_sitemap(sitemap_url: str = "https://ai-book-hackathon-mu.vercel.app/sitemap.xml") -> Set[str]:
    """
    Fetch and parse the sitemap XML to extract all URLs.

    Args:
        sitemap_url (str): URL of the sitemap to parse

    Returns:
        Set[str]: Set of all URLs found in the sitemap

    Raises:
        requests.RequestException: If unable to fetch the sitemap
        etree.XMLSyntaxError: If the sitemap XML is malformed
    """
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()
    except requests.RequestException as e:
        raise requests.RequestException(f"Failed to fetch sitemap from {sitemap_url}: {str(e)}")

    try:
        # Parse the XML
        root = etree.fromstring(response.content)

        # Define namespaces if present
        namespaces = {'sm': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        # Find all URL elements
        url_elements = root.xpath('//sm:url/sm:loc', namespaces=namespaces)

        # Extract the text content of each URL
        urls = {elem.text for elem in url_elements if elem.text and elem.text.strip()}

        return urls
    except etree.XMLSyntaxError as e:
        raise etree.XMLSyntaxError(f"Failed to parse sitemap XML: {str(e)}")


def compare_sitemap_with_stored_urls(sitemap_urls: Set[str], stored_urls: Set[str]) -> Dict[str, any]:
    """
    Compare sitemap URLs with URLs stored in Qdrant.

    Args:
        sitemap_urls (Set[str]): URLs from the sitemap
        stored_urls (Set[str]): URLs stored in Qdrant

    Returns:
        Dict: Comparison results including missing and extra URLs
    """
    missing_urls = sitemap_urls - stored_urls
    extra_urls = stored_urls - sitemap_urls

    completeness_percentage = 0
    if len(sitemap_urls) > 0:
        completeness_percentage = (len(stored_urls) - len(extra_urls)) / len(sitemap_urls) * 100

    return {
        'expected_urls': len(sitemap_urls),
        'stored_urls': len(stored_urls),
        'missing_urls': list(missing_urls),
        'extra_urls': list(extra_urls),
        'completeness_percentage': completeness_percentage
    }


def run_sample_similarity_query(query_text: str, top_k: int = 5, collection_name: str = "book_embeddings") -> List[Dict]:
    """
    Run a sample similarity query against the Qdrant collection.

    Args:
        query_text (str): Text to use for similarity search
        top_k (int): Number of results to return
        collection_name (str): Name of the Qdrant collection to query

    Returns:
        List[Dict]: List of similar content chunks with their metadata

    Raises:
        ConnectionError: If unable to connect to Qdrant
        Exception: If collection doesn't exist or other Qdrant errors occur
    """
    client = initialize_qdrant_client()

    try:
        # Check if collection exists
        client.get_collection(collection_name)
    except Exception as e:
        raise Exception(f"Collection '{collection_name}' does not exist or is not accessible: {str(e)}")

    # To properly search the embeddings, we need to embed the query text using the same model
    # that was used during ingestion. Since we're in the retrieval module, we need to import
    # the embedding functionality or implement it here.
    try:
        # Try to import and use the Cohere embedding functionality
        import os
        import cohere
        from dotenv import load_dotenv

        load_dotenv()

        # Initialize Cohere client
        cohere_api_key = os.getenv('COHERE_API_KEY')
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required for similarity search")

        cohere_client = cohere.Client(cohere_api_key)

        # Generate embedding for the query text
        response = cohere_client.embed(
            texts=[query_text],
            model='embed-multilingual-v3.0',  # Same model used during ingestion
            input_type="search_query"  # Appropriate for search queries
        )

        query_embedding = response.embeddings[0]

        # Perform vector search using the embedded query
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        results = []
        for result in search_results:
            results.append({
                'id': result.id,
                'payload': result.payload,
                'score': result.score
            })

        return results

    except ImportError:
        # If Cohere is not available, try other search methods
        print("Cohere library not available, trying alternative search methods...")
        try:
            # Try a keyword search approach if the collection supports it
            search_results = client.search(
                collection_name=collection_name,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_preview",
                            match=models.MatchText(text=query_text)
                        )
                    ]
                ),
                limit=top_k
            )

            results = []
            for result in search_results:
                results.append({
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score
                })

            return results
        except Exception:
            # If all search methods fail, return an empty list
            return []
    except Exception as e:
        # If Cohere embedding fails (e.g., no API key), try alternative methods
        print(f"Cohere embedding failed: {str(e)}, trying alternative search methods...")
        try:
            # Try a keyword search approach if the collection supports it
            search_results = client.search(
                collection_name=collection_name,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_preview",
                            match=models.MatchText(text=query_text)
                        )
                    ]
                ),
                limit=top_k
            )

            results = []
            for result in search_results:
                results.append({
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score
                })

            return results
        except Exception:
            # If all search methods fail, return an empty list
            return []


def format_query_results(results: List[Dict]) -> str:
    """
    Format and display query results with metadata.

    Args:
        results (List[Dict]): List of query results to format

    Returns:
        str: Formatted string representation of the results
    """
    if not results:
        return "No results found."

    formatted_results = []
    for i, result in enumerate(results, 1):
        payload = result.get('payload', {})
        text = payload.get('content_preview', '')[:200] + "..." if len(payload.get('content_preview', '')) > 200 else payload.get('content_preview', '')

        # Sanitize text to remove problematic Unicode characters
        text = text.encode('ascii', errors='ignore').decode('ascii', errors='ignore')

        formatted_result = f"""
Result {i} (Score: {result.get('score', 'N/A')}):
URL: {payload.get('url', 'N/A')}
Chunk Index: {payload.get('chunk_index', 'N/A')}
Text Preview: {text}
"""
        formatted_results.append(formatted_result)

    return "\n".join(formatted_results)


def execute_retrieval_verification() -> Dict[str, any]:
    """
    Execute all verification steps: retrieve content, validate metadata,
    compare URLs, and run sample queries.

    Returns:
        Dict: Complete verification results

    Raises:
        Exception: If any verification step fails
    """
    print("Starting book content retrieval verification...")

    try:
        # Step 1: Initialize Qdrant client
        print("Initializing Qdrant client...")
        client = initialize_qdrant_client()
        print("[OK] Qdrant client initialized")

        # Step 2: Retrieve all book chunks
        print("Retrieving all book content chunks from Qdrant...")
        chunks = retrieve_all_book_chunks()
        print(f"[OK] Retrieved {len(chunks)} content chunks")

        # Step 3: Validate metadata completeness
        print("Validating metadata completeness...")
        metadata_validation = validate_metadata_completeness(chunks)
        print(f"[OK] Metadata validation complete: {metadata_validation['valid_chunks']}/{metadata_validation['total_chunks']} valid chunks")

        # Step 4: Extract stored URLs
        stored_urls = {chunk['payload'].get('url') for chunk in chunks if chunk['payload'].get('url')}
        stored_urls.discard(None)  # Remove any None values

        # Step 5: Fetch and parse sitemap
        print("Fetching and parsing sitemap...")
        sitemap_urls = fetch_and_parse_sitemap()
        print(f"[OK] Sitemap parsed: {len(sitemap_urls)} URLs found")

        # Step 6: Compare sitemap with stored URLs
        print("Comparing sitemap URLs with stored URLs...")
        url_comparison = compare_sitemap_with_stored_urls(sitemap_urls, stored_urls)
        print(f"[OK] URL comparison complete: {url_comparison['completeness_percentage']:.2f}% completeness")

        # Step 7: Run sample query
        print("Running sample similarity query...")
        sample_query_results = run_sample_similarity_query("what is the name of the ai book")
        print(f"[OK] Sample query executed: {len(sample_query_results)} results")

        # Step 8: Format query results
        query_results_formatted = format_query_results(sample_query_results)

        # Compile all results
        verification_results = {
            'metadata_validation': metadata_validation,
            'url_comparison': url_comparison,
            'sample_query_results': sample_query_results,
            'query_results_formatted': query_results_formatted,
            'total_chunks_retrieved': len(chunks),
            'total_stored_urls': len(stored_urls),
            'total_sitemap_urls': len(sitemap_urls)
        }

        return verification_results
    except Exception as e:
        print(f"Error during verification: {str(e)}")
        raise


def main():
    """
    Main execution function that runs all verification steps.
    """
    try:
        results = execute_retrieval_verification()

        print("\n" + "="*60)
        print("BOOK CONTENT RETRIEVAL VERIFICATION RESULTS")
        print("="*60)

        print(f"\nTotal chunks retrieved: {results['total_chunks_retrieved']}")
        print(f"Total stored URLs: {results['total_stored_urls']}")
        print(f"Total sitemap URLs: {results['total_sitemap_urls']}")

        print(f"\nMetadata Validation:")
        print(f"  Valid chunks: {results['metadata_validation']['valid_chunks']}")
        print(f"  Invalid chunks: {results['metadata_validation']['invalid_chunks']}")

        print(f"\nURL Comparison:")
        print(f"  Completeness: {results['url_comparison']['completeness_percentage']:.2f}%")
        print(f"  Missing URLs: {len(results['url_comparison']['missing_urls'])}")
        print(f"  Extra URLs: {len(results['url_comparison']['extra_urls'])}")

        print(f"\nSample Query Results:")
        print(results['query_results_formatted'])

        if results['url_comparison']['missing_urls']:
            print(f"\nMissing URLs:")
            for url in results['url_comparison']['missing_urls'][:10]:  # Limit output
                print(f"  - {url}")
            if len(results['url_comparison']['missing_urls']) > 10:
                print(f"  ... and {len(results['url_comparison']['missing_urls']) - 10} more")

        print("\nVerification complete!")

    except Exception as e:
        print(f"Error during verification: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()