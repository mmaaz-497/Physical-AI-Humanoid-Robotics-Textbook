"""
Configuration management using Pydantic Settings.
Loads environment variables and provides typed configuration access.
"""

from pydantic_settings import BaseSettings
from typing import List, Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION: str

    # Gemini Configuration (OPTIONAL - only needed for answer generation)
    GEMINI_API_KEY: Optional[str] = None
    GEMINI_MODEL: str = "gemini-2.0-flash"

    # Database Configuration (OPTIONAL - only needed for chat logging)
    NEON_DATABASE_URL: Optional[str] = None

    # API Configuration
    CORS_ORIGINS: str = "http://localhost:3000"
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000

    # Retrieval Configuration
    SIMILARITY_THRESHOLD: float = 0.6
    MAX_TOP_K: int = 10
    DEFAULT_TOP_K: int = 5

    # Rate Limiting
    MAX_QUERY_LENGTH: int = 500
    MAX_SELECTION_TEXT_LENGTH: int = 5000

    class Config:
        env_file = ".env"
        case_sensitive = True

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.CORS_ORIGINS.split(",")]


# Global settings instance
settings = Settings()
