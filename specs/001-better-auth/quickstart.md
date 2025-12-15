# Quickstart: Better Auth Authentication System

**Feature**: Better Auth Authentication System
**Last Updated**: 2025-12-14

## Overview

This guide provides a quick reference for implementing and using the custom FastAPI authentication system for the Physical AI Humanoid Robotics documentation platform.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Backend Setup (FastAPI)](#backend-setup-fastapi)
3. [Frontend Integration (Docusaurus/React)](#frontend-integration-docusaurusreact)
4. [Testing the Implementation](#testing-the-implementation)
5. [Common Issues & Troubleshooting](#common-issues--troubleshooting)

---

## Prerequisites

**Required**:
- Python 3.11+
- PostgreSQL 14+ (existing Neon database)
- Node.js 18+ (existing for Docusaurus)
- Git

**Environment Variables**:
Create or update `backend/.env`:
```env
# Database (existing)
DATABASE_URL=postgresql://user:password@host:5432/dbname

# JWT Configuration (NEW)
JWT_SECRET_KEY=your-super-secret-256-bit-key-change-this-in-production
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=7

# Cookie Configuration (NEW)
COOKIE_DOMAIN=localhost  # Change to .yourdomain.com in production
COOKIE_SECURE=false      # Change to true in production (HTTPS)
COOKIE_SAMESITE=lax      # Change to none for cross-origin in production

# CORS (UPDATE existing)
CORS_ORIGINS=["http://localhost:3000", "http://localhost:8000"]
CORS_ALLOW_CREDENTIALS=true
```

---

## Backend Setup (FastAPI)

### Step 1: Install Dependencies

```bash
cd backend

# Add new dependencies to requirements.txt
echo "passlib[argon2]==1.7.4" >> requirements.txt
echo "python-jose[cryptography]==3.3.0" >> requirements.txt

# Install
pip install -r requirements.txt
```

### Step 2: Create Database Models

**File**: `backend/src/models/auth_models.py`

```python
import enum
import uuid
from datetime import datetime
from sqlalchemy import Column, String, Boolean, DateTime, Enum, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from backend.src.database import Base

class ExperienceLevel(str, enum.Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class ProfessionalRole(str, enum.Enum):
    STUDENT = "student"
    RESEARCHER = "researcher"
    ENGINEER = "engineer"
    HOBBYIST = "hobbyist"
    OTHER = "other"

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    hashed_password = Column(String(255), nullable=False)
    is_active = Column(Boolean, default=True, nullable=False, index=True)
    is_verified = Column(Boolean, default=False, nullable=False)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime(timezone=True), default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    last_login_at = Column(DateTime(timezone=True), nullable=True)

    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")
    refresh_tokens = relationship("RefreshToken", back_populates="user", cascade="all, delete-orphan")

class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), unique=True, nullable=False, index=True)
    experience_level = Column(Enum(ExperienceLevel), nullable=False, index=True)
    professional_role = Column(Enum(ProfessionalRole), nullable=False, index=True)
    role_other = Column(String(100), nullable=True)
    organization = Column(String(255), nullable=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime(timezone=True), default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    user = relationship("User", back_populates="profile")

class RefreshToken(Base):
    __tablename__ = "refresh_tokens"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    token = Column(String(255), unique=True, nullable=False, index=True)
    expires_at = Column(DateTime(timezone=True), nullable=False, index=True)
    revoked = Column(Boolean, default=False, nullable=False)
    revoked_at = Column(DateTime(timezone=True), nullable=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)

    user = relationship("User", back_populates="refresh_tokens")
```

### Step 3: Run Database Migrations

```bash
cd backend

# Create migration
alembic revision --autogenerate -m "Add authentication tables"

# Review migration file in backend/migrations/versions/

# Apply migration
alembic upgrade head
```

### Step 4: Create Security Utilities

**File**: `backend/src/utils/security.py`

```python
from datetime import datetime, timedelta
from jose import jwt, JWTError
from passlib.context import CryptContext
import secrets
from backend.src.config import settings

pwd_context = CryptContext(schemes=["argon2"], deprecated="auto")

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=30))
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)

def create_refresh_token() -> str:
    return secrets.token_urlsafe(32)

def decode_token(token: str) -> dict:
    try:
        payload = jwt.decode(token, settings.JWT_SECRET_KEY, algorithms=[settings.JWT_ALGORITHM])
        return payload
    except JWTError:
        return None
```

### Step 5: Create Authentication Router

**File**: `backend/src/routers/auth.py`

```python
from fastapi import APIRouter, Depends, HTTPException, status, Response
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from datetime import datetime, timedelta

from backend.src.database import get_db
from backend.src.models.auth_models import User, UserProfile, RefreshToken
from backend.src.utils.security import verify_password, get_password_hash, create_access_token, create_refresh_token, decode_token
from backend.src.config import settings
# Import your Pydantic schemas here

router = APIRouter(prefix="/auth", tags=["authentication"])
security = HTTPBearer()

@router.post("/signup", status_code=201)
async def signup(request: SignupRequest, response: Response, db: Session = Depends(get_db)):
    # Check if user exists
    existing_user = db.query(User).filter(User.email == request.email.lower()).first()
    if existing_user:
        raise HTTPException(status_code=409, detail="An account with this email already exists")

    # Create user
    hashed_password = get_password_hash(request.password)
    user = User(email=request.email.lower(), hashed_password=hashed_password)
    db.add(user)
    db.flush()

    # Create profile
    profile = UserProfile(
        user_id=user.id,
        experience_level=request.experience_level,
        professional_role=request.professional_role,
        role_other=request.role_other,
        organization=request.organization
    )
    db.add(profile)

    # Create refresh token
    token_value = create_refresh_token()
    refresh_token = RefreshToken(
        user_id=user.id,
        token=token_value,
        expires_at=datetime.utcnow() + timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)
    )
    db.add(refresh_token)
    db.commit()

    # Set cookie
    response.set_cookie(
        key="refresh_token",
        value=token_value,
        httponly=True,
        secure=settings.COOKIE_SECURE,
        samesite=settings.COOKIE_SAMESITE,
        max_age=settings.REFRESH_TOKEN_EXPIRE_DAYS * 24 * 60 * 60,
        path="/auth"
    )

    # Return access token
    access_token = create_access_token(data={"sub": str(user.id), "email": user.email})
    return AuthResponse(access_token=access_token, user=user, profile=profile)

@router.post("/login")
async def login(request: LoginRequest, response: Response, db: Session = Depends(get_db)):
    # Find user
    user = db.query(User).filter(User.email == request.email.lower()).first()
    if not user or not verify_password(request.password, user.hashed_password):
        raise HTTPException(status_code=401, detail="Invalid email or password")

    if not user.is_active:
        raise HTTPException(status_code=403, detail="Account is suspended")

    # Update last login
    user.last_login_at = datetime.utcnow()

    # Create refresh token
    token_value = create_refresh_token()
    refresh_token = RefreshToken(
        user_id=user.id,
        token=token_value,
        expires_at=datetime.utcnow() + timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)
    )
    db.add(refresh_token)
    db.commit()

    # Set cookie
    response.set_cookie(
        key="refresh_token",
        value=token_value,
        httponly=True,
        secure=settings.COOKIE_SECURE,
        samesite=settings.COOKIE_SAMESITE,
        max_age=settings.REFRESH_TOKEN_EXPIRE_DAYS * 24 * 60 * 60,
        path="/auth"
    )

    # Return access token
    access_token = create_access_token(data={"sub": str(user.id), "email": user.email})
    return AuthResponse(access_token=access_token, user=user, profile=user.profile)

@router.post("/logout", status_code=204)
async def logout(
    response: Response,
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
):
    # Decode token to get user_id
    payload = decode_token(credentials.credentials)
    if not payload:
        raise HTTPException(status_code=401, detail="Invalid token")

    # Revoke all refresh tokens
    db.query(RefreshToken).filter(
        RefreshToken.user_id == payload["sub"],
        RefreshToken.revoked == False
    ).update({"revoked": True, "revoked_at": datetime.utcnow()})
    db.commit()

    # Clear cookie
    response.delete_cookie(key="refresh_token", path="/auth")
    return Response(status_code=204)
```

### Step 6: Register Router in Main App

**File**: `backend/src/main.py`

```python
from fastapi import FastAPI
from backend.src.routers import auth  # Import auth router

app = FastAPI()

# Register auth router
app.include_router(auth.router)

# ... existing routes ...
```

### Step 7: Start Backend Server

```bash
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

---

## Frontend Integration (Docusaurus/React)

### Step 1: Create Auth Context

**File**: `src/contexts/AuthContext.tsx`

```typescript
import React, { createContext, useContext, useState, useEffect } from 'react';

interface User {
  id: string;
  email: string;
  is_active: boolean;
  created_at: string;
}

interface UserProfile {
  experience_level: string;
  professional_role: string;
  organization?: string;
}

interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  signup: (data: SignupData) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const API_BASE = process.env.NODE_ENV === 'production'
  ? 'https://api.yourdomain.com'
  : 'http://localhost:8000';

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState(true);
  const [accessToken, setAccessToken] = useState<string | null>(null);

  // Check if user is logged in on mount
  useEffect(() => {
    checkAuth();
  }, []);

  const checkAuth = async () => {
    try {
      const response = await fetch(`${API_BASE}/auth/me`, {
        credentials: 'include',
        headers: {
          ...(accessToken && { 'Authorization': `Bearer ${accessToken}` })
        }
      });

      if (response.status === 401) {
        // Try to refresh token
        const refreshed = await refreshAccessToken();
        if (!refreshed) {
          setUser(null);
          setProfile(null);
          return;
        }
        // Retry with new token
        return checkAuth();
      }

      if (response.ok) {
        const data = await response.json();
        setUser(data.user);
        setProfile(data.profile);
      }
    } catch (error) {
      console.error('Auth check failed:', error);
    } finally {
      setLoading(false);
    }
  };

  const refreshAccessToken = async (): Promise<boolean> => {
    try {
      const response = await fetch(`${API_BASE}/auth/refresh`, {
        method: 'POST',
        credentials: 'include'
      });

      if (response.ok) {
        const data = await response.json();
        setAccessToken(data.access_token);
        return true;
      }
    } catch (error) {
      console.error('Token refresh failed:', error);
    }
    return false;
  };

  const login = async (email: string, password: string) => {
    const response = await fetch(`${API_BASE}/auth/login`, {
      method: 'POST',
      credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password })
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Login failed');
    }

    const data = await response.json();
    setAccessToken(data.access_token);
    setUser(data.user);
    setProfile(data.profile);
  };

  const logout = async () => {
    try {
      await fetch(`${API_BASE}/auth/logout`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          ...(accessToken && { 'Authorization': `Bearer ${accessToken}` })
        }
      });
    } finally {
      setAccessToken(null);
      setUser(null);
      setProfile(null);
    }
  };

  const signup = async (data: SignupData) => {
    const response = await fetch(`${API_BASE}/auth/signup`, {
      method: 'POST',
      credentials: 'include',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signup failed');
    }

    const responseData = await response.json();
    setAccessToken(responseData.access_token);
    setUser(responseData.user);
    setProfile(responseData.profile);
  };

  return (
    <AuthContext.Provider value={{ user, profile, loading, login, logout, signup }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
}
```

### Step 2: Wrap App with AuthProvider

**File**: `src/theme/Root.tsx`

```tsx
import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}
```

### Step 3: Create Login/Signup Pages

**File**: `src/pages/auth/login.tsx`

```tsx
import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

export default function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login } = useAuth();
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    try {
      await login(email, password);
      history.push('/');
    } catch (err) {
      setError(err.message);
    }
  };

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <h1>Sign In</h1>
          {error && <div className="alert alert--danger">{error}</div>}
          <form onSubmit={handleSubmit}>
            <div className="margin-bottom--md">
              <label htmlFor="email">Email</label>
              <input
                id="email"
                type="email"
                className="form-control"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
              />
            </div>
            <div className="margin-bottom--md">
              <label htmlFor="password">Password</label>
              <input
                id="password"
                type="password"
                className="form-control"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
              />
            </div>
            <button type="submit" className="button button--primary">
              Sign In
            </button>
          </form>
        </div>
      </div>
    </div>
  );
}
```

---

## Testing the Implementation

### Backend API Tests

```bash
# Test signup
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecureP@ssw0rd",
    "experience_level": "intermediate",
    "professional_role": "student"
  }'

# Test login
curl -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "SecureP@ssw0rd"}' \
  -c cookies.txt

# Test /auth/me (replace TOKEN with actual token from login)
curl -X GET http://localhost:8000/auth/me \
  -H "Authorization: Bearer <TOKEN>" \
  -b cookies.txt
```

### Frontend Integration Tests

1. Navigate to `http://localhost:3000/auth/login`
2. Enter credentials and click "Sign In"
3. Verify redirect to homepage
4. Check that user email appears in navigation
5. Click logout and verify session cleared

---

## Common Issues & Troubleshooting

### CORS Errors

**Problem**: Browser shows CORS error when calling API

**Solution**:
```python
# backend/src/main.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Cookies Not Being Set

**Problem**: Refresh token cookie not appearing in browser

**Solution**:
- Check `COOKIE_SECURE=false` in development (localhost)
- Check `COOKIE_SAMESITE=lax` in development
- Ensure `credentials: 'include'` in frontend fetch calls

### JWT Decode Error

**Problem**: "Invalid token" error

**Solution**:
- Verify `JWT_SECRET_KEY` matches in `.env`
- Check token expiration (default 30 min)
- Ensure token format is `Bearer <token>`

### Database Migration Fails

**Problem**: Alembic migration fails with "table already exists"

**Solution**:
```bash
# Drop tables and recreate
alembic downgrade base
alembic upgrade head
```

---

## Next Steps

1. **Add Rate Limiting**: Implement `slowapi` for login/signup endpoints
2. **Add Email Verification**: Create email verification workflow
3. **Add Password Reset**: Implement forgot password flow
4. **Add OAuth Providers**: Integrate Google, GitHub sign-in
5. **Add 2FA**: Implement two-factor authentication
6. **Deploy to Production**: Configure HTTPS, update CORS, set secure cookies

---

## Support

For issues or questions:
- Check API documentation: `http://localhost:8000/docs`
- Review implementation plan: `specs/001-better-auth/plan.md`
- Review data models: `specs/001-better-auth/data-model.md`
