"""
Test script to verify the RAG agent works with a simple test
"""
import sys
import os
from pathlib import Path

# Add the parent directory to the path to import modules
current_dir = Path(__file__).parent
parent_dir = current_dir.parent
sys.path.insert(0, str(parent_dir))

from agent import RAGAgent

def test_rag_agent():
    print("Testing RAG Agent directly...")
    try:
        agent = RAGAgent()
        result = agent.process_query_sync("What is this book about?")
        print("RAG Agent response:", result)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_rag_agent()