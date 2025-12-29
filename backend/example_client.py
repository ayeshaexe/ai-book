"""
Example client code demonstrating how to interact with the Book RAG API.

This script shows how to make requests to the /query endpoint and handle responses.
"""
import requests
import json
from typing import Dict, Any


def query_book_content(query: str, api_url: str = "http://localhost:8000") -> Dict[Any, Any]:
    """
    Query the book content API with a given question.

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
        response.raise_for_status()  # Raises an HTTPError for bad responses

        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error making request: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"Response content: {e.response.text}")
        return {"error": str(e)}


def main():
    """Example usage of the Book RAG API client."""
    print("Book RAG API Client Example")
    print("=" * 30)

    # Example queries
    queries = [
        "What is the main concept of this book?",
        "How many modules are there?",
        "Explain physical AI concepts"
    ]

    for query in queries:
        print(f"\nQuery: {query}")
        print("-" * 40)

        result = query_book_content(query)

        if "error" in result:
            print(f"Error: {result['error']}")
        else:
            print(f"Answer: {result.get('answer', 'No answer')}")
            print(f"Confidence: {result.get('confidence', 'N/A')}")
            print(f"Sources: {len(result.get('sources', []))} source(s) found")

            # Print first source if available
            sources = result.get('sources', [])
            if sources:
                first_source = sources[0]
                print(f"First source preview: {first_source.get('content', '')[:100]}...")


if __name__ == "__main__":
    main()