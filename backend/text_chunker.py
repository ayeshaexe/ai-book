import re
from typing import List, Dict
import logging

logger = logging.getLogger(__name__)

def chunk_text(content: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[Dict]:
    """
    Split content into semantically meaningful segments.

    Args:
        content (str): The content to chunk
        max_chunk_size (int): Maximum size of each chunk in characters
        overlap (int): Number of characters to overlap between chunks

    Returns:
        List[Dict]: List of chunks with metadata
    """
    if not content:
        return []

    # Split content by paragraphs first
    paragraphs = content.split('\n\n')

    chunks = []
    chunk_index = 0

    current_chunk = ""

    for paragraph in paragraphs:
        # If adding this paragraph would exceed the chunk size
        if len(current_chunk) + len(paragraph) > max_chunk_size:
            # If current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append({
                    'content': current_chunk.strip(),
                    'chunk_index': chunk_index,
                    'length': len(current_chunk)
                })
                chunk_index += 1

            # If the paragraph itself is larger than max_chunk_size, split it
            if len(paragraph) > max_chunk_size:
                sub_chunks = _split_large_paragraph(paragraph, max_chunk_size, overlap)
                for sub_chunk in sub_chunks:
                    chunks.append({
                        'content': sub_chunk.strip(),
                        'chunk_index': chunk_index,
                        'length': len(sub_chunk)
                    })
                    chunk_index += 1
                current_chunk = ""
            else:
                current_chunk = paragraph
        else:
            current_chunk += "\n\n" + paragraph if current_chunk else paragraph

    # Add the last chunk if it exists
    if current_chunk.strip():
        chunks.append({
            'content': current_chunk.strip(),
            'chunk_index': chunk_index,
            'length': len(current_chunk)
        })

    logger.info(f"Content chunked into {len(chunks)} chunks")
    return chunks

def _split_large_paragraph(paragraph: str, max_chunk_size: int, overlap: int) -> List[str]:
    """
    Split a large paragraph into smaller chunks based on sentences.

    Args:
        paragraph (str): Large paragraph to split
        max_chunk_size (int): Maximum size of each chunk
        overlap (int): Number of characters to overlap

    Returns:
        List[str]: List of sub-chunks
    """
    # Split by sentences
    sentences = re.split(r'[.!?]+\s+', paragraph)

    sub_chunks = []
    current_sub_chunk = ""

    for sentence in sentences:
        # If adding this sentence would exceed the chunk size
        if len(current_sub_chunk) + len(sentence) > max_chunk_size:
            # If current sub-chunk is not empty, save it
            if current_sub_chunk.strip():
                sub_chunks.append(current_sub_chunk.strip())

            # If the sentence itself is larger than max_chunk_size, split it by words
            if len(sentence) > max_chunk_size:
                word_chunks = _split_large_sentence(sentence, max_chunk_size, overlap)
                sub_chunks.extend(word_chunks)
                current_sub_chunk = ""
            else:
                current_sub_chunk = sentence
        else:
            current_sub_chunk += " " + sentence if current_sub_chunk else sentence

    # Add the last sub-chunk if it exists
    if current_sub_chunk.strip():
        sub_chunks.append(current_sub_chunk.strip())

    return sub_chunks

def _split_large_sentence(sentence: str, max_chunk_size: int, overlap: int) -> List[str]:
    """
    Split a large sentence into smaller chunks based on words.

    Args:
        sentence (str): Large sentence to split
        max_chunk_size (int): Maximum size of each chunk
        overlap (int): Number of characters to overlap

    Returns:
        List[str]: List of sub-chunks
    """
    words = sentence.split()
    chunks = []
    current_chunk = ""

    for word in words:
        if len(current_chunk) + len(word) > max_chunk_size:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
                # Create overlapping chunk if possible
                if overlap > 0 and len(current_chunk) > overlap:
                    current_chunk = current_chunk[-overlap:] + " " + word
                else:
                    current_chunk = word
            else:
                current_chunk = word
        else:
            current_chunk += " " + word if current_chunk else word

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks

def chunk_text_semantic(content: str, max_chunk_size: int = 1000) -> List[Dict]:
    """
    Alternative semantic chunking approach that tries to maintain context boundaries.

    Args:
        content (str): The content to chunk
        max_chunk_size (int): Maximum size of each chunk in characters

    Returns:
        List[Dict]: List of chunks with metadata
    """
    if not content:
        return []

    # Try to split by semantic boundaries: headings, paragraphs, sentences
    # First, try to find headings
    sections = re.split(r'(?=\n#+\s|\n<h[1-6][^>]*>|\n\s*\*{3,}\s*\n)', content)

    chunks = []
    chunk_index = 0

    for section in sections:
        if not section.strip():
            continue

        # If section is too large, further chunk it
        if len(section) > max_chunk_size:
            # Split by paragraphs within the section
            paragraphs = re.split(r'\n\s*\n', section)
            temp_chunk = ""

            for paragraph in paragraphs:
                if len(temp_chunk) + len(paragraph) <= max_chunk_size:
                    temp_chunk += "\n\n" + paragraph if temp_chunk else paragraph
                else:
                    if temp_chunk.strip():
                        chunks.append({
                            'content': temp_chunk.strip(),
                            'chunk_index': chunk_index,
                            'length': len(temp_chunk)
                        })
                        chunk_index += 1
                    temp_chunk = paragraph

            if temp_chunk.strip():
                chunks.append({
                    'content': temp_chunk.strip(),
                    'chunk_index': chunk_index,
                    'length': len(temp_chunk)
                })
                chunk_index += 1
        else:
            chunks.append({
                'content': section.strip(),
                'chunk_index': chunk_index,
                'length': len(section)
            })
            chunk_index += 1

    logger.info(f"Content semantically chunked into {len(chunks)} chunks")
    return chunks

if __name__ == "__main__":
    # Example usage
    sample_content = """
    This is the first paragraph of our content. It contains some important information about the topic we're discussing.

    This is the second paragraph. It continues the discussion and provides additional details that are relevant to understanding the concept.

    # Section Title
    This paragraph is under a section title. The chunking algorithm should try to respect these semantic boundaries when possible.

    Here's another paragraph in the same section. It provides more information related to the section topic.
    """

    chunks = chunk_text(sample_content, max_chunk_size=100)
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i}: {len(chunk['content'])} chars - {chunk['content'][:50]}...")