"""
Qdrant vector database service for semantic search.
"""

import logging
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.models import SearchRequest, PointStruct
from fastembed import TextEmbedding
import time

from src.config import settings
from src.models.retrieval import RetrievedChunk
from src.utils.exceptions import QdrantConnectionError

logger = logging.getLogger(__name__)


class QdrantService:
    """Service for interacting with Qdrant vector database."""

    def __init__(self):
        """Initialize Qdrant client and FastEmbed model for embeddings."""
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30.0,  # Increased timeout
        )
        self.collection_name = settings.QDRANT_COLLECTION
        # Use the same embedding model as the upload script
        self.embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
        logger.info(f"Qdrant service initialized for collection: {self.collection_name}")

    def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding vector for user query using FastEmbed (BAAI/bge-small-en-v1.5).

        Args:
            query: The user's question

        Returns:
            384-dimensional embedding vector (matching Qdrant collection)

        Raises:
            QdrantConnectionError: If embedding generation fails
        """
        try:
            # Generate embedding using FastEmbed (same model as upload script)
            embeddings = list(self.embedding_model.embed([query]))
            embedding = embeddings[0].tolist()
            logger.debug(f"Generated embedding for query: {query[:50]}...")
            return embedding
        except Exception as e:
            logger.error(f"Failed to generate query embedding: {str(e)}")
            raise QdrantConnectionError(f"Failed to generate embedding: {str(e)}")

    def search(
        self,
        query: str,
        top_k: int = 5,
        similarity_threshold: float = None,
    ) -> List[RetrievedChunk]:
        """
        Search Qdrant collection for relevant chunks using semantic similarity.

        Args:
            query: The user's question
            top_k: Number of chunks to retrieve
            similarity_threshold: Minimum similarity score (default from settings)

        Returns:
            List of retrieved chunks sorted by similarity score

        Raises:
            QdrantConnectionError: If search fails
        """
        if similarity_threshold is None:
            similarity_threshold = settings.SIMILARITY_THRESHOLD

        max_retries = 3
        retry_delay = 1.0

        for attempt in range(max_retries):
            try:
                # Generate query embedding
                query_vector = self.generate_query_embedding(query)

                # Search Qdrant using query_points for newer API
                from qdrant_client.models import FieldCondition, Filter, MatchValue

                search_result = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_vector,
                    limit=top_k,
                    score_threshold=similarity_threshold,
                ).points

                # Convert to RetrievedChunk objects
                chunks = []
                for hit in search_result:
                    chunk = RetrievedChunk(
                        qdrant_id=str(hit.id),
                        chunk_text=hit.payload.get("text", ""),
                        similarity_score=hit.score,
                        metadata=hit.payload,
                    )
                    chunks.append(chunk)

                logger.info(
                    f"Retrieved {len(chunks)} chunks for query (threshold={similarity_threshold})"
                )
                return chunks

            except Exception as e:
                if attempt < max_retries - 1:
                    logger.warning(
                        f"Qdrant search failed (attempt {attempt + 1}/{max_retries}): {str(e)}"
                    )
                    time.sleep(retry_delay * (2 ** attempt))  # Exponential backoff
                else:
                    logger.error(f"Qdrant search failed after {max_retries} attempts: {str(e)}")
                    raise QdrantConnectionError(
                        f"Failed to search Qdrant after {max_retries} attempts: {str(e)}"
                    )

        return []

    def test_connection(self) -> bool:
        """
        Test connection to Qdrant collection.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Qdrant connection OK. Collection has {collection_info.points_count} points")
            return True
        except Exception as e:
            logger.error(f"Qdrant connection test failed: {str(e)}")
            return False
