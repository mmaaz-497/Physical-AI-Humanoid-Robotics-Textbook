"""
Database configuration and session management.
"""

from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from typing import Generator, Optional

from src.config import settings

# Base class for declarative models (always needed for imports)
Base = declarative_base()

# Conditionally create database engine and session
if settings.NEON_DATABASE_URL:
    # Create SQLAlchemy engine
    engine = create_engine(
        settings.NEON_DATABASE_URL,
        pool_pre_ping=True,  # Enable connection health checks
        pool_size=5,
        max_overflow=10,
    )
    # Create session factory
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
else:
    engine = None
    SessionLocal = None


def get_db() -> Generator[Optional[Session], None, None]:
    """
    Dependency that provides a database session.
    Automatically closes the session after use.
    Returns None if database is not configured.
    """
    if SessionLocal is None:
        yield None
        return

    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
