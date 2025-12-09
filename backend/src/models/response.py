"""
Pydantic response models for API endpoints.
"""

from pydantic import BaseModel, Field
from typing import List


class CitationSource(BaseModel):
    """Citation information for a source chunk."""

    chapter: str = Field(..., description="Chapter title where the information was found")
    file_path: str = Field(..., description="File path to the chapter (e.g., /docs/ros2/introduction)")
    chunk_text: str = Field(..., description="Excerpt from the source text")


class QueryResponse(BaseModel):
    """Response model for the /query endpoint."""

    answer: str = Field(..., description="The generated answer to the user's question")
    sources: List[CitationSource] = Field(
        default_factory=list,
        description="List of source citations used to generate the answer",
    )
    confidence: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence score based on similarity scores (0.0-1.0)",
    )
    used_selection_text: bool = Field(
        default=False,
        description="Whether the answer was generated from highlighted text",
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "answer": "ROS 2 is a robotics middleware that provides libraries and tools for building robot applications. (Source: Module 1: ROS 2 Fundamentals)",
                    "sources": [
                        {
                            "chapter": "Module 1: ROS 2 Fundamentals",
                            "file_path": "/docs/ros2/introduction",
                            "chunk_text": "ROS 2 is a robotics middleware..."
                        }
                    ],
                    "confidence": 0.89,
                    "used_selection_text": False
                }
            ]
        }
    }


class HealthResponse(BaseModel):
    """Response model for health check endpoint."""

    status: str = Field(default="ok", description="Health status")
    version: str = Field(default="1.0.0", description="API version")
