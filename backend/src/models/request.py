"""
Pydantic request models for API endpoints.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional


class QueryRequest(BaseModel):
    """Request model for the /query endpoint."""

    q: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="The user's question (max 500 characters)",
        examples=["What is ROS 2?"],
    )

    top_k: int = Field(
        default=5,
        ge=1,
        le=10,
        description="Number of chunks to retrieve from Qdrant (1-10)",
    )

    selection_text: Optional[str] = Field(
        default=None,
        max_length=5000,
        description="Optional highlighted text to use as context (bypasses Qdrant retrieval)",
    )

    @field_validator("q")
    @classmethod
    def validate_question(cls, v: str) -> str:
        """Validate and clean the question."""
        v = v.strip()
        if not v:
            raise ValueError("Question cannot be empty")
        return v

    @field_validator("selection_text")
    @classmethod
    def validate_selection_text(cls, v: Optional[str]) -> Optional[str]:
        """Validate and clean selection text."""
        if v is not None:
            v = v.strip()
            if not v:
                return None
        return v

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "q": "What is ROS 2?",
                    "top_k": 5
                },
                {
                    "q": "Explain this code",
                    "selection_text": "ROS 2 nodes are independent processes that communicate via topics.",
                    "top_k": 3
                }
            ]
        }
    }
