"""
Data models for retrieval operations.
"""

from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class RetrievedChunk:
    """
    Represents a chunk of text retrieved from Qdrant vector database.
    """

    qdrant_id: str
    chunk_text: str
    similarity_score: float
    metadata: Dict[str, Any]

    def __repr__(self) -> str:
        return f"<RetrievedChunk(id={self.qdrant_id}, score={self.similarity_score:.3f})>"
