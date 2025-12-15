"""
JWT Authentication Middleware for validating Better Auth tokens.
"""
from fastapi import HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import jwt, JWTError
from typing import Optional
import os

from ..models.auth_schemas import TokenPayload, CurrentUser

# Security scheme for JWT tokens
security = HTTPBearer(auto_error=False)

# Get JWT secret from environment
BETTER_AUTH_JWT_SECRET = os.getenv("BETTER_AUTH_JWT_SECRET")

if not BETTER_AUTH_JWT_SECRET:
    print("WARNING: BETTER_AUTH_JWT_SECRET not set. JWT validation will fail.")


async def get_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> CurrentUser:
    """
    Validate JWT token and return current user.

    Raises:
        HTTPException: 401 if token is invalid or missing

    Returns:
        CurrentUser: Authenticated user data from JWT
    """
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = credentials.credentials

    try:
        # Decode JWT token
        payload = jwt.decode(
            token,
            BETTER_AUTH_JWT_SECRET,
            algorithms=["HS256"],
            options={"verify_exp": True}
        )

        # Extract user ID from 'sub' claim
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token: missing user ID",
            )

        # Create token payload
        token_data = TokenPayload(**payload)

    except JWTError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Invalid token: {str(e)}",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Return current user with extracted data
    return CurrentUser(
        id=user_id,
        email=payload.get("email"),
        experience_level=payload.get("experienceLevel"),
        professional_role=payload.get("professionalRole"),
        role_other=payload.get("roleOther"),
        organization=payload.get("organization"),
    )


async def get_current_user_optional(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[CurrentUser]:
    """
    Validate JWT token if present, return None if not authenticated.

    This allows endpoints to be accessible to both authenticated and
    unauthenticated users, while still tracking authenticated users.

    Returns:
        Optional[CurrentUser]: User data if authenticated, None otherwise
    """
    if not credentials:
        return None

    try:
        return await get_current_user(credentials)
    except HTTPException:
        # Token invalid or expired - treat as unauthenticated
        return None
