import time
import logging
import requests
from functools import wraps
from typing import Callable, Any
import random

logger = logging.getLogger(__name__)

def retry_with_backoff(
    max_retries: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    backoff_factor: float = 2.0,
    exceptions: tuple = (Exception,)
):
    """
    Decorator to retry a function with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts
        base_delay: Initial delay between retries in seconds
        max_delay: Maximum delay between retries in seconds
        backoff_factor: Factor by which delay increases after each retry
        exceptions: Tuple of exceptions to catch and retry on
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt == max_retries:
                        # Final attempt failed
                        logger.error(f"Function {func.__name__} failed after {max_retries} retries: {e}")
                        raise e

                    # Calculate delay with exponential backoff and jitter
                    delay = min(base_delay * (backoff_factor ** attempt), max_delay)
                    jitter = random.uniform(0, 0.1 * delay)  # Add up to 10% jitter
                    total_delay = delay + jitter

                    logger.warning(
                        f"Attempt {attempt + 1} failed for {func.__name__}: {e}. "
                        f"Retrying in {total_delay:.2f} seconds..."
                    )
                    time.sleep(total_delay)

            # This line should never be reached, but included for type safety
            raise last_exception

        return wrapper
    return decorator

def is_network_error(exception: Exception) -> bool:
    """
    Determine if an exception is a network-related error that should be retried.

    Args:
        exception: The exception to check

    Returns:
        bool: True if the exception is network-related, False otherwise
    """
    if isinstance(exception, requests.RequestException):
        return True
    if isinstance(exception, ConnectionError):
        return True
    if "timeout" in str(exception).lower():
        return True
    if "connection" in str(exception).lower():
        return True
    if "network" in str(exception).lower():
        return True
    return False

def is_api_error(exception: Exception) -> bool:
    """
    Determine if an exception is an API-related error that should be retried.

    Args:
        exception: The exception to check

    Returns:
        bool: True if the exception is API-related, False otherwise
    """
    # Check if it's a Cohere API error
    if "cohere" in str(type(exception)).lower():
        error_msg = str(exception).lower()
        if any(error in error_msg for error in ["rate limit", "quota", "api", "502", "503", "504"]):
            return True

    # Check for HTTP status codes that suggest retry
    if hasattr(exception, 'response') and hasattr(exception.response, 'status_code'):
        status_code = exception.response.status_code
        # Retry on server errors and rate limit errors
        if status_code in [429, 500, 502, 503, 504]:
            return True

    return False

def is_retryable_error(exception: Exception) -> bool:
    """
    Determine if an exception is retryable.

    Args:
        exception: The exception to check

    Returns:
        bool: True if the exception is retryable, False otherwise
    """
    return is_network_error(exception) or is_api_error(exception)

def safe_request(url: str, **kwargs) -> requests.Response:
    """
    Make a safe request with retry logic.

    Args:
        url: URL to request
        **kwargs: Additional arguments for requests.get()

    Returns:
        requests.Response: The response object
    """
    @retry_with_backoff(
        max_retries=3,
        exceptions=(requests.RequestException, ConnectionError)
    )
    def _make_request():
        response = requests.get(url, **kwargs)
        response.raise_for_status()
        return response

    return _make_request()

def safe_cohere_call(func, *args, **kwargs):
    """
    Make a safe Cohere API call with retry logic.

    Args:
        func: The Cohere API function to call
        *args: Arguments to pass to the function
        **kwargs: Keyword arguments to pass to the function

    Returns:
        The result of the Cohere API call
    """
    @retry_with_backoff(
        max_retries=3,
        exceptions=(Exception,)  # Catch all exceptions for Cohere calls
    )
    def _make_cohere_call():
        result = func(*args, **kwargs)
        return result

    return _make_cohere_call()

if __name__ == "__main__":
    # Example usage of the retry decorator
    @retry_with_backoff(
        max_retries=3,
        exceptions=(ConnectionError, requests.RequestException)
    )
    def unstable_function():
        import random
        if random.random() < 0.7:  # 70% chance of failure
            raise ConnectionError("Simulated network error")
        return "Success!"

    try:
        result = unstable_function()
        print(f"Function succeeded: {result}")
    except Exception as e:
        print(f"Function failed after retries: {e}")