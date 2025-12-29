"""
Test script to verify the retrieve_book_content function works
"""
import sys
import os
from pathlib import Path

# Add the parent directory to the path to import modules
current_dir = Path(__file__).parent
parent_dir = current_dir.parent
sys.path.insert(0, str(parent_dir))

from agent import retrieve_book_content

def test_retrieve_function():
    print("Testing retrieve_book_content function directly...")
    try:
        result = retrieve_book_content("What is this book about?")
        print("Retrieve function response:", result)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_retrieve_function()