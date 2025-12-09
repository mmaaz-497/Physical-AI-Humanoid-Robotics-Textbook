"""
SQLAlchemy ORM models for database tables.
"""

from sqlalchemy import Column, Integer, String, Text, DateTime, JSON, func
from src.database import Base


class ChatLog(Base):
    """
    Stores all chat interactions for analytics and debugging.
    """

    __tablename__ = "chat_logs"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    question = Column(Text, nullable=False)
    answer = Column(Text, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    retrieval_metadata = Column(JSON, nullable=True)  # Stores Qdrant chunk IDs, similarity scores
    model_used = Column(String(50), nullable=False)  # e.g., "gpt-4o-mini"
    session_id = Column(String(255), nullable=True)  # Optional session tracking

    def __repr__(self):
        return f"<ChatLog(id={self.id}, timestamp={self.timestamp})>"


class BookIndex(Base):
    """
    Maps Qdrant vector IDs to book chapter information for citation generation.
    """

    __tablename__ = "book_index"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    qdrant_id = Column(String(255), unique=True, nullable=False, index=True)
    chapter_title = Column(String(255), nullable=False)
    file_path = Column(String(500), nullable=False)  # e.g., /docs/ros2/introduction
    section_heading = Column(String(255), nullable=True)  # Optional subsection

    def __repr__(self):
        return f"<BookIndex(qdrant_id={self.qdrant_id}, chapter={self.chapter_title})>"
