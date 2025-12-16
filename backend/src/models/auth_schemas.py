"""
Authentication Schemas for JWT validation and user data.
"""
from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime


class TokenPayload(BaseModel):
    """JWT token payload schema."""
    sub: str  # User ID
    email: Optional[EmailStr] = None
    exp: Optional[int] = None  # Expiration timestamp
    iat: Optional[int] = None  # Issued at timestamp


class CurrentUser(BaseModel):
    """Current authenticated user data from JWT."""
    id: str
    email: Optional[EmailStr] = None
    experience_level: Optional[str] = None
    professional_role: Optional[str] = None
    role_other: Optional[str] = None
    organization: Optional[str] = None


class UserResponse(BaseModel):
    """User response schema for API responses."""
    id: str
    email: str
    is_active: bool = True
    is_verified: bool = False
    created_at: datetime
    last_login_at: Optional[datetime] = None
    experience_level: Optional[str] = None
    professional_role: Optional[str] = None
    role_other: Optional[str] = None
    organization: Optional[str] = None

    class Config:
        from_attributes = True
