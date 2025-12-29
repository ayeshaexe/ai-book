"""
Test script to verify client-server communication with the Book RAG API.

This script tests the communication between client and server components.
"""
import requests
import json
from typing import Dict, Any


def test_api_connection(api_url: str = "http://localhost:8000") -> bool:
    """
    Test basic API connection by hitting the root endpoint.

    Args:
        api_url: The base URL of the API (default: http://localhost:8000)

    Returns:
        True if connection is successful, False otherwise
    """
    try:
        response = requests.get(f"{api_url}/")
        response.raise_for_status()
        data = response.json()
        print(f"API connection successful: {data}")
        return True
    except requests.exceptions.RequestException as e:
        print(f"API connection failed: {e}")
        return False


def test_query_endpoint(query: str, api_url: str = "http://localhost:8000") -> Dict[Any, Any]:
    """
    Test the query endpoint with a sample query.

    Args:
        query: The question to ask about the book content
        api_url: The base URL of the API (default: http://localhost:8000)

    Returns:
        The JSON response from the API
    """
    url = f"{api_url}/query"

    payload = {
        "query": query
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        response = requests.post(url, json=payload, headers=headers)
        response.raise_for_status()

        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Query request failed: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"Response status code: {e.response.status_code}")
            print(f"Response content: {e.response.text}")
        return {"error": str(e)}


def main():
    """Test client-server communication."""
    print("Testing Client-Server Communication")
    print("=" * 40)

    api_url = "http://localhost:8000"  # Default API URL

    # Test basic connection
    print("1. Testing basic API connection...")
    if test_api_connection(api_url):
        print("✓ Connection to API successful\n")
    else:
        print("✗ Connection to API failed")
        print("Make sure the FastAPI server is running on http://localhost:8000")
        return

    # Test query endpoint
    print("2. Testing query endpoint...")
    test_queries = [
        "What is this book about?",
        "How many modules does this book contain?"
    ]

    for i, query in enumerate(test_queries, 1):
        print(f"\n2.{i} Testing query: '{query}'")
        result = test_query_endpoint(query, api_url)

        if "error" in result:
            print(f"✗ Query failed: {result['error']}")
        else:
            print(f"✓ Query successful")
            print(f"  Answer preview: {result.get('answer', 'N/A')[:100]}...")
            print(f"  Sources found: {len(result.get('sources', []))}")
            print(f"  Confidence: {result.get('confidence', 'N/A')}")

    print(f"\n3. Testing error handling with empty query...")
    error_result = test_query_endpoint("", api_url)
    if "error" in error_result:
        print("✓ Error handling works correctly")
    else:
        print("? Unexpected response for empty query")

    print("\nClient-Server Communication Test Complete!")


if __name__ == "__main__":
    main()