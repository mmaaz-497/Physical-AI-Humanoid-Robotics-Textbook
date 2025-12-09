"""
Grounding service for citation generation and out-of-scope detection.
"""

import logging
from typing import List
from sqlalchemy.orm import Session

from src.models.retrieval import RetrievedChunk
from src.models.response import CitationSource
from src.models.database import BookIndex
from src.config import settings
from src.utils.exceptions import OutOfScopeQueryError

logger = logging.getLogger(__name__)


class GroundingService:
    """Service for validating answer grounding and generating citations."""

    def is_out_of_scope(
        self,
        context_chunks: List[RetrievedChunk],
        similarity_threshold: float = None,
    ) -> bool:
        """
        Detect if query is out of scope based on retrieval results.

        Args:
            context_chunks: Retrieved chunks from Qdrant
            similarity_threshold: Minimum acceptable similarity (default from settings)

        Returns:
            True if query is out of scope, False otherwise
        """
        if similarity_threshold is None:
            similarity_threshold = settings.SIMILARITY_THRESHOLD

        # No chunks retrieved = out of scope
        if not context_chunks:
            logger.warning("No chunks retrieved - query is out of scope")
            return True

        # All chunks below threshold = out of scope
        if all(chunk.similarity_score < similarity_threshold for chunk in context_chunks):
            logger.warning(
                f"All chunks below threshold {similarity_threshold} - query is out of scope"
            )
            return True

        return False

    def generate_refusal_message(self) -> str:
        """
        Generate user-friendly refusal message for out-of-scope queries.

        Returns:
            Refusal message string
        """
        return (
            "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content. "
            "Your question appears to be outside the book's scope. "
            "Please try asking about topics covered in the textbook."
        )

    def generate_citations(
        self,
        context_chunks: List[RetrievedChunk],
        db: Session,
    ) -> List[CitationSource]:
        """
        Generate citation sources from retrieved chunks by looking up book index.

        Args:
            context_chunks: Retrieved chunks from Qdrant
            db: Database session

        Returns:
            List of CitationSource objects
        """
        citations = []

        for chunk in context_chunks:
            # Lookup chapter info from book_index
            book_entry = (
                db.query(BookIndex)
                .filter(BookIndex.qdrant_id == chunk.qdrant_id)
                .first()
            )

            if book_entry:
                citation = CitationSource(
                    chapter=book_entry.chapter_title,
                    file_path=book_entry.file_path,
                    chunk_text=chunk.chunk_text[:200] + "..."  # Truncate for brevity
                    if len(chunk.chunk_text) > 200
                    else chunk.chunk_text,
                )
                citations.append(citation)
            else:
                # Fallback if book_index entry not found
                logger.warning(f"Book index entry not found for Qdrant ID: {chunk.qdrant_id}")
                citation = CitationSource(
                    chapter="Unknown Chapter",
                    file_path="/docs/unknown",
                    chunk_text=chunk.chunk_text[:200] + "..."
                    if len(chunk.chunk_text) > 200
                    else chunk.chunk_text,
                )
                citations.append(citation)

        logger.info(f"Generated {len(citations)} citations")
        return citations

    def calculate_confidence(self, context_chunks: List[RetrievedChunk]) -> float:
        """
        Calculate confidence score based on similarity scores of retrieved chunks.

        Args:
            context_chunks: Retrieved chunks from Qdrant

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not context_chunks:
            return 0.0

        # Use average of top 3 similarity scores
        top_scores = sorted(
            [chunk.similarity_score for chunk in context_chunks],
            reverse=True,
        )[:3]

        if not top_scores:
            return 0.0

        confidence = sum(top_scores) / len(top_scores)
        return min(max(confidence, 0.0), 1.0)  # Clamp to [0, 1]
