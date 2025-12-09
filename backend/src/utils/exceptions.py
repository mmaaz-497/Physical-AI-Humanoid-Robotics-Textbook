"""
Custom exception classes for the RAG chatbot application.
"""


class QdrantConnectionError(Exception):
    """Raised when connection to Qdrant fails or times out."""

    def __init__(self, message: str = "Failed to connect to Qdrant vector database"):
        self.message = message
        super().__init__(self.message)


class OpenAIRateLimitError(Exception):
    """Raised when OpenAI API rate limit is exceeded."""

    def __init__(self, message: str = "OpenAI API rate limit exceeded", retry_after: int = None):
        self.message = message
        self.retry_after = retry_after
        super().__init__(self.message)


class OutOfScopeQueryError(Exception):
    """Raised when a query is outside the scope of the textbook content."""

    def __init__(self, message: str = "Query is outside the scope of the textbook"):
        self.message = message
        super().__init__(self.message)


class DatabaseError(Exception):
    """Raised when database operations fail."""

    def __init__(self, message: str = "Database operation failed"):
        self.message = message
        super().__init__(self.message)


class InvalidQueryError(Exception):
    """Raised when query validation fails."""

    def __init__(self, message: str = "Invalid query parameters"):
        self.message = message
        super().__init__(self.message)
