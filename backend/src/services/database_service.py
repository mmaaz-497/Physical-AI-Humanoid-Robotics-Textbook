"""
Database service for chat logging and book index queries.
"""

import logging
from datetime import datetime
from typing import Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError

from src.models.database import ChatLog, BookIndex
from src.utils.exceptions import DatabaseError

logger = logging.getLogger(__name__)


class DatabaseService:
    """Service for database operations."""

    def insert_chat_log(
        self,
        db: Session,
        question: str,
        answer: str,
        retrieval_metadata: Optional[Dict[str, Any]],
        model_used: str,
        session_id: Optional[str] = None,
    ) -> ChatLog:
        """
        Insert a chat log entry into the database.

        Args:
            db: Database session
            question: User's question
            answer: Generated answer
            retrieval_metadata: JSON metadata about retrieved chunks
            model_used: Name of the model used (e.g., "gpt-4o-mini")
            session_id: Optional session identifier

        Returns:
            Created ChatLog object

        Raises:
            DatabaseError: If insertion fails
        """
        try:
            chat_log = ChatLog(
                question=question,
                answer=answer,
                retrieval_metadata=retrieval_metadata,
                model_used=model_used,
                session_id=session_id,
            )
            db.add(chat_log)
            db.commit()
            db.refresh(chat_log)

            logger.info(f"Chat log inserted with ID: {chat_log.id}")
            return chat_log

        except SQLAlchemyError as e:
            db.rollback()
            logger.error(f"Failed to insert chat log: {str(e)}")
            raise DatabaseError(f"Failed to log chat interaction: {str(e)}")

    def query_book_index(
        self,
        db: Session,
        qdrant_id: str,
    ) -> Optional[BookIndex]:
        """
        Query book index by Qdrant ID.

        Args:
            db: Database session
            qdrant_id: Qdrant vector ID

        Returns:
            BookIndex object if found, None otherwise
        """
        try:
            book_entry = (
                db.query(BookIndex)
                .filter(BookIndex.qdrant_id == qdrant_id)
                .first()
            )
            return book_entry

        except SQLAlchemyError as e:
            logger.error(f"Failed to query book index: {str(e)}")
            return None

    def get_chat_history(
        self,
        db: Session,
        limit: int = 100,
        session_id: Optional[str] = None,
    ) -> list[ChatLog]:
        """
        Retrieve recent chat history.

        Args:
            db: Database session
            limit: Maximum number of logs to retrieve
            session_id: Optional session filter

        Returns:
            List of ChatLog objects
        """
        try:
            query = db.query(ChatLog)

            if session_id:
                query = query.filter(ChatLog.session_id == session_id)

            logs = query.order_by(ChatLog.timestamp.desc()).limit(limit).all()
            return logs

        except SQLAlchemyError as e:
            logger.error(f"Failed to retrieve chat history: {str(e)}")
            return []
