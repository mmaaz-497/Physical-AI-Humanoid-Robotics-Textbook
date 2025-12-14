"""
FastAPI main application for the RAG Chatbot API.
"""
import uvicorn
import logging
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from src.config import settings
from src.routers import query
from src.models.response import HealthResponse
from src.utils import logging as app_logging
from src.utils.exceptions import (
    QdrantConnectionError,
    OpenAIRateLimitError,
    OutOfScopeQueryError,
    DatabaseError,
    InvalidQueryError,
)

logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(query.router)


# Global exception handlers
@app.exception_handler(QdrantConnectionError)
async def qdrant_connection_error_handler(request, exc: QdrantConnectionError):
    """Handle Qdrant connection errors."""
    logger.error(f"Qdrant connection error: {exc.message}")
    return JSONResponse(
        status_code=503,
        content={
            "detail": "Vector database temporarily unavailable. Please try again later.",
            "error_type": "QdrantConnectionError",
        },
    )


@app.exception_handler(OpenAIRateLimitError)
async def openai_rate_limit_error_handler(request, exc: OpenAIRateLimitError):
    """Handle Gemini rate limit errors."""
    logger.error(f"Gemini rate limit error: {exc.message}")
    return JSONResponse(
        status_code=429,
        content={
            "detail": "Service temporarily unavailable due to high demand. Please try again in a moment.",
            "error_type": "OpenAIRateLimitError",
            "retry_after": exc.retry_after,
        },
    )


@app.exception_handler(OutOfScopeQueryError)
async def out_of_scope_error_handler(request, exc: OutOfScopeQueryError):
    """Handle out-of-scope query errors."""
    logger.warning(f"Out of scope query: {exc.message}")
    return JSONResponse(
        status_code=200,  # Not an error, just out of scope
        content={
            "answer": exc.message,
            "sources": [],
            "confidence": 0.0,
            "used_selection_text": False,
        },
    )


@app.exception_handler(DatabaseError)
async def database_error_handler(request, exc: DatabaseError):
    """Handle database errors."""
    logger.error(f"Database error: {exc.message}")
    return JSONResponse(
        status_code=500,
        content={
            "detail": "Database error occurred. Your query may not have been logged.",
            "error_type": "DatabaseError",
        },
    )


@app.exception_handler(InvalidQueryError)
async def invalid_query_error_handler(request, exc: InvalidQueryError):
    """Handle invalid query errors."""
    logger.warning(f"Invalid query: {exc.message}")
    return JSONResponse(
        status_code=400,
        content={
            "detail": exc.message,
            "error_type": "InvalidQueryError",
        },
    )


# Health check endpoint
@app.get("/health", response_model=HealthResponse, tags=["health"])
async def health_check():
    """
    Health check endpoint.

    Returns:
        HealthResponse with status and version
    """
    logger.debug("Health check requested")
    return HealthResponse(status="ok", version="1.0.0")


# Root endpoint
@app.get("/", tags=["root"])
async def root():
    """
    Root endpoint with API information.

    Returns:
        JSON with API info and links
    """
    return {
        "message": "Physical AI Textbook RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health",
        "query_endpoint": "/api/query",
    }


# Startup event
@app.on_event("startup")
async def startup_event():
    """Application startup - deferred connection validation to save memory."""
    logger.info("Starting Physical AI RAG Chatbot API")
    logger.info(f"CORS origins: {settings.cors_origins_list}")

    # Connection tests deferred to first request to avoid loading heavy models at startup
    # This keeps memory usage low on free-tier deployments (e.g., Render 512MB limit)
    logger.info("Deferring service initialization to first request (memory optimization)")
    logger.info("API startup complete")


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Application shutdown."""
    logger.info("Shutting down Physical AI RAG Chatbot API")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.main:app",
        host=settings.API_HOST,
        port=settings.API_PORT,
        reload=False,
        log_level="info",
        
    )
