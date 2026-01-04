
"""
RAG Agent Implementation
Retrieval-Augmented Generation (RAG) agent using:
- OpenAI Agents SDK
- OpenRouter (no OpenAI API key required)
- Qdrant (book_embeddings)
"""

import os
import time
import asyncio
import logging
from typing import Dict, Any
from dotenv import load_dotenv

from agents.agent import Agent
from agents.run import Runner
from agents.tool import function_tool
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel

from qdrant_client import QdrantClient
from openai import OpenAI, AsyncOpenAI

# ---------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------

load_dotenv()

OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION = "book_embeddings"

if not OPENROUTER_API_KEY:
    raise RuntimeError("OPENROUTER_API_KEY is not set")

# ---------------------------------------------------------------------
# OpenRouter client (used for both chat + embeddings)
# ---------------------------------------------------------------------

# Create OpenAI client with OpenRouter configuration
client = OpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1",
)

# Create model configuration for OpenRouter (using AsyncOpenAI for the model)
async_client = AsyncOpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1",
)

# Create model configuration for OpenRouter
model = OpenAIChatCompletionsModel(
    openai_client=async_client,
    model="mistralai/devstral-2512:free",  # Example model, replace with a working one
)

# ---------------------------------------------------------------------
# Tool: Retrieve content from Qdrant
# ---------------------------------------------------------------------

@function_tool
def retrieve_book_content(query: str, top_k: int = 5) -> Dict[str, Any]:
    """
    Retrieve relevant book chunks from Qdrant.
    """
    try:
        import cohere
        import os

        # Use Cohere to create embeddings (same as used during ingestion)
        cohere_api_key = os.getenv('COHERE_API_KEY')
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required for similarity search")

        cohere_client = cohere.Client(cohere_api_key)

        # Generate embedding for the query text
        response = cohere_client.embed(
            texts=[query],
            model='embed-multilingual-v3.0',  # Same model used during ingestion
            input_type="search_query"  # Appropriate for search queries
        )

        query_embedding = response.embeddings[0]

        # Initialize Qdrant client for this function
        if QDRANT_URL and QDRANT_API_KEY:
            # Use cloud Qdrant instance
            qdrant = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
            )
        else:
            # Use local in-memory Qdrant instance
            qdrant = QdrantClient(":memory:")

        # Perform vector search using the embedded query
        search_results = qdrant.search(
            collection_name=QDRANT_COLLECTION,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Extract the text content from the search results
        chunks = []
        for result in search_results:
            if result.payload:
                # Try different possible keys for the text content
                text_content = result.payload.get("content_preview") or result.payload.get("text") or result.payload.get("content") or ""
                if text_content:
                    chunks.append({
                        "id": result.id,
                        "content": text_content,
                        "score": result.score,
                    })

        if not chunks:
            return {
                "chunks": [],
                "context": "No relevant content found in the book.",
            }

        return {
            "chunks": chunks,
            "context": "\n".join(c["content"] for c in chunks),
        }
    except Exception as e:
        logging.error(f"Error in retrieve_book_content tool: {str(e)}")
        return {
            "chunks": [],
            "context": f"Error retrieving content: {str(e)}"
        }

# ---------------------------------------------------------------------
# RAG Agent
# ---------------------------------------------------------------------

class RAGAgent:
    def __init__(self):
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        self.agent = Agent(
            name="BookRAGAgent",
            instructions=(
               "Answer questions using ONLY the retrieved book content. "
               "You MUST always call retrieve_book_content before answering. "

               "If relevant information is found, answer strictly from that content. "

               "If retrieve_book_content fails due to an error (e.g., rate limit 429) "
               "AND the question is clearly similar to the book's main topics, "
               "you may provide a best-effort answer based on general understanding "
               "of the book, but you MUST clearly prefix the answer with: "
               "'[Inferred due to retrieval error]'. "

               "If the answer is not found and cannot be reasonably inferred, reply exactly: "
               "'Not found in the book.' "

               "Never hallucinate specific facts, quotes, page numbers, or details."
            ),
            tools=[retrieve_book_content],
            model=model,
        )

    async def process_query(self, query: str) -> Dict[str, Any]:
        start = time.time()

        if not query.strip():
            return {"response": "Invalid query"}

        try:
            result = await Runner.run(self.agent, query)
            response_text = result.final_output
        except Exception as e:
            logging.error(f"Error running agent: {str(e)}")
            response_text = f"I encountered an issue processing your query: {str(e)}. Please check that the required services are available and configured correctly."

        return {
            "response": response_text,
            "latency": round(time.time() - start, 2),
        }

    def process_query_sync(self, query: str) -> Dict[str, Any]:
        import asyncio
        import threading

        # Check if we're already in an event loop
        try:
            loop = asyncio.get_running_loop()
            # We're already in a loop, run in a separate thread
            result = None
            exception = None

            def run_in_thread():
                nonlocal result, exception
                try:
                    thread_loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(thread_loop)
                    try:
                        result = thread_loop.run_until_complete(self.process_query(query))
                    finally:
                        thread_loop.close()
                except Exception as e:
                    exception = e

            thread = threading.Thread(target=run_in_thread)
            thread.start()
            thread.join()

            if exception:
                raise exception
            return result
        except RuntimeError:
            # No event loop running, safe to create one
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    return loop.run_until_complete(self.process_query(query))
                finally:
                    loop.close()
            except Exception as e:
                logging.error(f"Error in synchronous query processing: {str(e)}")
                return {
                    "response": f"I encountered an issue processing your query: {str(e)}. Please check that the required services are available and configured correctly.",
                    "latency": 0.0
                }

# ---------------------------------------------------------------------
# Example
# ---------------------------------------------------------------------

if __name__ == "__main__":
    agent = RAGAgent()
    output = agent.process_query_sync("how many modules are there?")
    print(output)
