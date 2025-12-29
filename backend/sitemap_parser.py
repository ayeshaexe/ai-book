import requests
from lxml import etree
from typing import List
import logging

logger = logging.getLogger(__name__)

def parse_sitemap(sitemap_url: str) -> List[str]:
    """
    Fetch and parse the sitemap to extract all book page URLs.

    Args:
        sitemap_url (str): URL to the sitemap.xml file

    Returns:
        List[str]: List of page URLs extracted from the sitemap
    """
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        # Parse the XML content
        xml_content = response.content
        root = etree.fromstring(xml_content)

        # Define the namespace map to handle potential namespaces
        namespaces = {
            'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
            # Add other common sitemap namespaces if needed
        }

        # Find all URL elements in the sitemap
        urls = []
        for url_elem in root.xpath('//sitemap:url/sitemap:loc', namespaces=namespaces):
            urls.append(url_elem.text)

        # If no namespace found, try without namespace
        if not urls:
            urls = [elem.text for elem in root.xpath('//url/loc')]

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        logger.error(f"Error fetching sitemap: {e}")
        raise
    except etree.XMLSyntaxError as e:
        logger.error(f"Error parsing sitemap XML: {e}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error parsing sitemap: {e}")
        raise

def fetch_and_parse_sitemap(sitemap_url: str) -> List[str]:
    """
    Public interface to fetch and parse sitemap.

    Args:
        sitemap_url (str): URL to the sitemap.xml file

    Returns:
        List[str]: List of page URLs extracted from the sitemap
    """
    return parse_sitemap(sitemap_url)

if __name__ == "__main__":
    # Example usage
    sitemap_url = "https://ai-book-hackathon-mu.vercel.app/sitemap.xml"
    urls = fetch_and_parse_sitemap(sitemap_url)
    print(f"Discovered {len(urls)} URLs:")
    for url in urls:
        print(url)