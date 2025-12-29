import requests
import json
import time

def test_query_endpoint():
    """
    Test the query endpoint to ensure it works without errors
    """
    # URL for the query endpoint (assuming the server is running on localhost:8000)
    url = "http://localhost:8000/query"

    # Sample query to test
    test_query = {
        "query": "What is this book about?",
        "context": "General question about the book content",
        "user_id": "test_user_123"
    }

    print("Testing query endpoint...")
    print(f"Sending query: {test_query['query']}")

    try:
        response = requests.post(
            url,
            headers={"Content-Type": "application/json"},
            data=json.dumps(test_query)
        )

        print(f"Response status code: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"Response: {json.dumps(response_data, indent=2)}")
            print("SUCCESS: Query endpoint test PASSED")
        else:
            print(f"FAILED: Query endpoint test FAILED with status code: {response.status_code}")
            print(f"Error response: {response.text}")

    except requests.exceptions.ConnectionError:
        print("FAILED: Connection error: The server might not be running.")
        print("Please start the server with: uvicorn backend.frontend:app --reload")
    except Exception as e:
        print(f"FAILED: Error during test: {str(e)}")

def test_empty_query():
    """
    Test the query endpoint with an empty query to ensure validation works
    """
    url = "http://localhost:8000/query"

    empty_query = {
        "query": "",
        "context": "Testing empty query validation",
        "user_id": "test_user_123"
    }

    print("\nTesting empty query validation...")

    try:
        response = requests.post(
            url,
            headers={"Content-Type": "application/json"},
            data=json.dumps(empty_query)
        )

        print(f"Response status code for empty query: {response.status_code}")

        if response.status_code == 400:
            print("SUCCESS: Empty query validation works correctly")
        else:
            print(f"FAILED: Empty query validation failed. Expected 400, got {response.status_code}")

    except Exception as e:
        print(f"FAILED: Error during empty query test: {str(e)}")

def test_long_query():
    """
    Test the query endpoint with a very long query to ensure length validation works
    """
    url = "http://localhost:8000/query"

    long_query = {
        "query": "This is a very long query that exceeds the maximum allowed length. " * 20,
        "context": "Testing long query validation",
        "user_id": "test_user_123"
    }

    print("\nTesting long query validation...")

    try:
        response = requests.post(
            url,
            headers={"Content-Type": "application/json"},
            data=json.dumps(long_query)
        )

        print(f"Response status code for long query: {response.status_code}")

        if response.status_code == 400:
            print("SUCCESS: Long query validation works correctly")
        else:
            print(f"FAILED: Long query validation failed. Expected 400, got {response.status_code}")

    except Exception as e:
        print(f"FAILED: Error during long query test: {str(e)}")

if __name__ == "__main__":
    print("Running endpoint tests...")
    test_query_endpoint()
    test_empty_query()
    test_long_query()
    print("\nAll tests completed!")