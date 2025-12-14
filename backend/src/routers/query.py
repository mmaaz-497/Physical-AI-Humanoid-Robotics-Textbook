"""
Query router for the RAG chatbot API.
"""

import logging
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from src.database import get_db
from src.models.request import QueryRequest
from src.models.response import QueryResponse
from src.services.qdrant_service import QdrantService
from src.services.openai_service import OpenAIService
from src.services.grounding_service import GroundingService
from src.services.database_service import DatabaseService
from src.utils.exceptions import (
    QdrantConnectionError,
    OpenAIRateLimitError,
    OutOfScopeQueryError,
)
from src.config import settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["query"])

# Initialize services (singleton pattern)
qdrant_service = QdrantService()
openai_service = OpenAIService() if settings.GEMINI_API_KEY else None
grounding_service = GroundingService()
database_service = DatabaseService()


@router.post("/query", response_model=QueryResponse)
async def query_chatbot(
    request: QueryRequest,
    db: Optional[Session] = Depends(get_db),
) -> QueryResponse:
    """
    Query endpoint for the RAG chatbot.

    Retrieves relevant context from Qdrant, generates grounded answer using OpenAI,
    and logs the interaction to the database.

    Args:
        request: QueryRequest containing question, top_k, and optional selection_text
        db: Database session dependency

    Returns:
        QueryResponse with answer, citations, and confidence score

    Raises:
        HTTPException: 503 if Qdrant/OpenAI unavailable, 429 if rate limited, 400 if invalid query
    """
    logger.info(f"Received query: {request.q[:50]}...")

    try:
        # Case 1: Selection text provided - bypass Qdrant retrieval
        if request.selection_text:
            logger.info("Using selection text mode (bypassing Qdrant)")

            # If OpenAI available, use it; otherwise return selection text
            if openai_service:
                answer = openai_service.generate_answer_from_selection(
                    question=request.q,
                    selection_text=request.selection_text,
                )
            else:
                answer = f"**Selected Text:**\n\n{request.selection_text}\n\n*(Retrieval-only mode: OpenAI not configured)*"

            # Log to database (if configured)
            if settings.NEON_DATABASE_URL:
                try:
                    database_service.insert_chat_log(
                        db=db,
                        question=request.q,
                        answer=answer,
                        retrieval_metadata={"mode": "selection_text", "text_length": len(request.selection_text)},
                        model_used=settings.GEMINI_MODEL if openai_service else "retrieval-only",
                    )
                except Exception as db_error:
                    logger.error(f"Failed to log chat (non-fatal): {str(db_error)}")

            return QueryResponse(
                answer=answer,
                sources=[],  # No sources in selection mode
                confidence=1.0,  # High confidence when using user-selected text
                used_selection_text=True,
            )

        # Case 2: Normal RAG flow - retrieve from Qdrant
        context_chunks = qdrant_service.search(
            query=request.q,
            top_k=request.top_k,
            similarity_threshold=settings.SIMILARITY_THRESHOLD,
        )

        # Check if query is out of scope
        if grounding_service.is_out_of_scope(context_chunks):
            refusal_message = grounding_service.generate_refusal_message()

            # Log refusal (if database configured)
            if settings.NEON_DATABASE_URL:
                try:
                    database_service.insert_chat_log(
                        db=db,
                        question=request.q,
                        answer=refusal_message,
                        retrieval_metadata={"out_of_scope": True, "chunks_retrieved": 0},
                        model_used=settings.GEMINI_MODEL if openai_service else "retrieval-only",
                    )
                except Exception as db_error:
                    logger.error(f"Failed to log chat (non-fatal): {str(db_error)}")

            return QueryResponse(
                answer=refusal_message,
                sources=[],
                confidence=0.0,
                used_selection_text=False,
            )

        # Generate answer from retrieved context
        if openai_service:
            # Use OpenAI to generate natural language answer
            answer = openai_service.generate_answer(
                question=request.q,
                context_chunks=context_chunks,
            )
        else:
            # Retrieval-only mode: Format raw chunks as answer
            answer = "**Retrieved book content:**\n\n"
            for i, chunk in enumerate(context_chunks[:3], 1):  # Show top 3 chunks
                answer += f"**Result {i}** (similarity: {chunk.similarity_score:.2f}):\n{chunk.chunk_text}\n\n"
            answer += f"\n*(Retrieval-only mode: {len(context_chunks)} chunks found. OpenAI not configured for answer generation.)*"

        # Generate citations (if database available)
        citations = []
        if db:
            citations = grounding_service.generate_citations(
                context_chunks=context_chunks,
                db=db,
            )

        # Calculate confidence
        confidence = grounding_service.calculate_confidence(context_chunks)

        # Log to database (if configured)
        if settings.NEON_DATABASE_URL:
            try:
                retrieval_metadata = {
                    "chunks_retrieved": len(context_chunks),
                    "top_k": request.top_k,
                    "similarity_scores": [chunk.similarity_score for chunk in context_chunks],
                    "qdrant_ids": [chunk.qdrant_id for chunk in context_chunks],
                }

                database_service.insert_chat_log(
                    db=db,
                    question=request.q,
                    answer=answer,
                    retrieval_metadata=retrieval_metadata,
                    model_used=settings.GEMINI_MODEL if openai_service else "retrieval-only",
                )
            except Exception as db_error:
                logger.error(f"Failed to log chat (non-fatal): {str(db_error)}")

        return QueryResponse(
            answer=answer,
            sources=citations,
            confidence=confidence,
            used_selection_text=False,
        )

    except QdrantConnectionError as e:
        logger.error(f"Qdrant connection error: {str(e)}")
        raise HTTPException(
            status_code=503,
            detail="Vector database temporarily unavailable. Please try again later.",
        )

    except OpenAIRateLimitError as e:
        logger.error(f"OpenAI rate limit error: {str(e)}")
        raise HTTPException(
            status_code=429,
            detail="Service temporarily unavailable due to high demand. Please try again in a moment.",
        )

    except Exception as e:
        logger.error(f"Unexpected error in query endpoint: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="An unexpected error occurred. Please try again.",
        )
