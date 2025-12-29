import requests
from bs4 import BeautifulSoup
from typing import Tuple
import logging

logger = logging.getLogger(__name__)

def extract_content(page_url: str) -> Tuple[str, str]:
    """
    Extract clean text content from a book page.

    Args:
        page_url (str): URL of the book page to extract content from

    Returns:
        Tuple[str, str]: (page_title, clean_content_text)
    """
    try:
        response = requests.get(page_url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract the page title
        title_tag = soup.find('title')
        title = title_tag.get_text().strip() if title_tag else "No Title"

        # Remove unwanted elements (navigation, headers, footers, etc.)
        for element in soup(['script', 'style', 'nav', 'header', 'footer', 'aside', 'advertisement', 'ad']):
            element.decompose()

        # Try to find the main content area
        # Common selectors for main content
        main_content = None
        for selector in ['main', 'article', '[role="main"]', '.content', '.main-content', '.post-content', '.entry-content']:
            main_content = soup.select_one(selector)
            if main_content:
                break

        # If no main content area found, use the body
        if not main_content:
            main_content = soup.find('body')

        # Extract text from the main content
        if main_content:
            content_text = main_content.get_text(separator=' ', strip=True)
        else:
            content_text = soup.get_text(separator=' ', strip=True)

        # Clean up the text by removing extra whitespace
        import re
        content_text = re.sub(r'\s+', ' ', content_text)

        logger.info(f"Extracted content from {page_url} - Title: {title[:50]}...")
        return title, content_text

    except requests.RequestException as e:
        logger.error(f"Error fetching page {page_url}: {e}")
        raise
    except Exception as e:
        logger.error(f"Error extracting content from {page_url}: {e}")
        raise

def clean_content(content: str) -> str:
    """
    Additional cleaning of extracted content.

    Args:
        content (str): Raw extracted content

    Returns:
        str: Cleaned content
    """
    # Remove extra whitespace
    import re
    cleaned = re.sub(r'\s+', ' ', content)
    return cleaned.strip()

if __name__ == "__main__":
    # Example usage
    test_url = "https://ai-book-hackathon-mu.vercel.app/"
    title, content = extract_content(test_url)
    print(f"Title: {title}")
    print(f"Content preview: {content[:500]}...")