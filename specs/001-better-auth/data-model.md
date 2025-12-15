# Data Model: Better Auth Authentication System

**Feature**: Better Auth Authentication System
**Date**: 2025-12-14
**Branch**: `001-better-auth`

## Overview

This document defines the data models for the authentication system. All models use SQLAlchemy ORM with PostgreSQL backend and include Pydantic schemas for API validation.

## Entity Relationship Diagram

```
┌─────────────────┐
│     User        │
│                 │
│ PK id           │
│    email        │◄──────────┐
│    hashed_pwd   │           │ FK user_id
│    is_active    │           │ (one-to-one)
│    is_verified  │           │
│    created_at   │      ┌────┴────────────┐
│    updated_at   │      │  UserProfile    │
│    last_login   │      │                 │
└────────┬────────┘      │ PK id           │
         │               │ FK user_id      │
         │               │    exp_level    │
         │ FK user_id    │    prof_role    │
         │ (one-to-many) │    role_other   │
         │               │    organization │
    ┌────▼────────────┐  │    created_at   │
    │ RefreshToken    │  │    updated_at   │
    │                 │  └─────────────────┘
    │ PK id           │
    │ FK user_id      │
    │    token        │
    │    expires_at   │
    │    revoked      │
    │    revoked_at   │
    │    created_at   │
    └─────────────────┘
```

## Database Tables

### 1. users

**Purpose**: Core authentication and user account management

**Schema**:
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL UNIQUE,
    hashed_password VARCHAR(255) NOT NULL,
    is_active BOOLEAN NOT NULL DEFAULT TRUE,
    is_verified BOOLEAN NOT NULL DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    last_login_at TIMESTAMP WITH TIME ZONE
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_is_active ON users(is_active);
```

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique user identifier |
| `email` | VARCHAR(255) | UNIQUE, NOT NULL | User email address (login identifier) |
| `hashed_password` | VARCHAR(255) | NOT NULL | Argon2id hashed password |
| `is_active` | BOOLEAN | NOT NULL, DEFAULT TRUE | Account active status (for soft delete/suspension) |
| `is_verified` | BOOLEAN | NOT NULL, DEFAULT FALSE | Email verification status (future use) |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Account creation timestamp |
| `updated_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |
| `last_login_at` | TIMESTAMP | NULLABLE | Last successful login timestamp |

**Validation Rules**:
- Email must be valid format (RFC 5322)
- Email must be lowercase (normalized)
- Password must meet strength requirements before hashing
- Hashed password must be stored, never plain text

**SQLAlchemy Model**:
```python
from sqlalchemy import Column, String, Boolean, DateTime
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

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

    # Relationships
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")
    refresh_tokens = relationship("RefreshToken", back_populates="user", cascade="all, delete-orphan")
```

---

### 2. user_profiles

**Purpose**: Store user background information for content personalization

**Schema**:
```sql
CREATE TYPE experience_level_enum AS ENUM ('beginner', 'intermediate', 'advanced');
CREATE TYPE professional_role_enum AS ENUM ('student', 'researcher', 'engineer', 'hobbyist', 'other');

CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    experience_level experience_level_enum NOT NULL,
    professional_role professional_role_enum NOT NULL,
    role_other VARCHAR(100),
    organization VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_profiles_exp_level ON user_profiles(experience_level);
CREATE INDEX idx_user_profiles_prof_role ON user_profiles(professional_role);
```

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique profile identifier |
| `user_id` | UUID | FK, UNIQUE, NOT NULL | Reference to users.id (one-to-one) |
| `experience_level` | ENUM | NOT NULL | User's experience level with Physical AI/Robotics |
| `professional_role` | ENUM | NOT NULL | User's professional role/use case |
| `role_other` | VARCHAR(100) | NULLABLE | Custom role description (when professional_role = 'other') |
| `organization` | VARCHAR(255) | NULLABLE | User's organization/affiliation (optional) |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Profile creation timestamp |
| `updated_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |

**Enum Values**:

**experience_level**:
- `beginner` - New to the field
- `intermediate` - Some hands-on experience
- `advanced` - Professional or research experience

**professional_role**:
- `student` - Learning for academic purposes
- `researcher` - Conducting research in robotics/AI
- `engineer` - Working in industry
- `hobbyist` - Personal interest and projects
- `other` - Custom role (must provide role_other)

**Validation Rules**:
- If professional_role = 'other', role_other is REQUIRED
- If professional_role != 'other', role_other should be NULL
- organization is always optional

**SQLAlchemy Model**:
```python
import enum
from sqlalchemy import Column, String, Enum, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship

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

    # Relationships
    user = relationship("User", back_populates="profile")
```

---

### 3. refresh_tokens

**Purpose**: Manage refresh tokens for session management and revocation

**Schema**:
```sql
CREATE TABLE refresh_tokens (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) NOT NULL UNIQUE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    revoked BOOLEAN NOT NULL DEFAULT FALSE,
    revoked_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_refresh_tokens_user_id ON refresh_tokens(user_id);
CREATE INDEX idx_refresh_tokens_token ON refresh_tokens(token);
CREATE INDEX idx_refresh_tokens_expires_at ON refresh_tokens(expires_at);
```

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Unique token record identifier |
| `user_id` | UUID | FK, NOT NULL | Reference to users.id |
| `token` | VARCHAR(255) | UNIQUE, NOT NULL | Refresh token value (hashed or random string) |
| `expires_at` | TIMESTAMP | NOT NULL | Token expiration timestamp |
| `revoked` | BOOLEAN | NOT NULL, DEFAULT FALSE | Token revocation status |
| `revoked_at` | TIMESTAMP | NULLABLE | Timestamp when token was revoked |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Token creation timestamp |

**Validation Rules**:
- Token must be unique across all records
- expires_at must be in the future when created
- If revoked = TRUE, revoked_at must be set
- Token cannot be used if revoked = TRUE or expires_at < NOW()

**SQLAlchemy Model**:
```python
from sqlalchemy import Column, String, Boolean, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship

class RefreshToken(Base):
    __tablename__ = "refresh_tokens"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    token = Column(String(255), unique=True, nullable=False, index=True)
    expires_at = Column(DateTime(timezone=True), nullable=False, index=True)
    revoked = Column(Boolean, default=False, nullable=False)
    revoked_at = Column(DateTime(timezone=True), nullable=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)

    # Relationships
    user = relationship("User", back_populates="refresh_tokens")
```

---

## Pydantic Schemas

### Request Schemas

**SignupRequest**:
```python
from pydantic import BaseModel, EmailStr, Field, validator

class SignupRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8, max_length=128)
    experience_level: ExperienceLevel
    professional_role: ProfessionalRole
    role_other: Optional[str] = Field(None, max_length=100)
    organization: Optional[str] = Field(None, max_length=255)

    @validator('role_other')
    def validate_role_other(cls, v, values):
        if values.get('professional_role') == ProfessionalRole.OTHER and not v:
            raise ValueError('role_other is required when professional_role is "other"')
        if values.get('professional_role') != ProfessionalRole.OTHER and v:
            raise ValueError('role_other should only be provided when professional_role is "other"')
        return v
```

**LoginRequest**:
```python
class LoginRequest(BaseModel):
    email: EmailStr
    password: str
```

### Response Schemas

**UserResponse**:
```python
class UserResponse(BaseModel):
    id: UUID
    email: str
    is_active: bool
    is_verified: bool
    created_at: datetime
    last_login_at: Optional[datetime]

    class Config:
        from_attributes = True
```

**UserProfileResponse**:
```python
class UserProfileResponse(BaseModel):
    experience_level: ExperienceLevel
    professional_role: ProfessionalRole
    role_other: Optional[str]
    organization: Optional[str]
    created_at: datetime

    class Config:
        from_attributes = True
```

**AuthResponse**:
```python
class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserResponse
    profile: Optional[UserProfileResponse]
```

**MeResponse**:
```python
class MeResponse(BaseModel):
    user: UserResponse
    profile: Optional[UserProfileResponse]
```

---

## Database Migrations

**Alembic Migration** (`alembic/versions/001_create_auth_tables.py`):

```python
"""Create authentication tables

Revision ID: 001
Revises:
Create Date: 2025-12-14
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '001'
down_revision = None
branch_labels = None
depends_on = None

def upgrade():
    # Create enums
    experience_level_enum = postgresql.ENUM('beginner', 'intermediate', 'advanced', name='experience_level_enum')
    professional_role_enum = postgresql.ENUM('student', 'researcher', 'engineer', 'hobbyist', 'other', name='professional_role_enum')
    experience_level_enum.create(op.get_bind())
    professional_role_enum.create(op.get_bind())

    # Create users table
    op.create_table(
        'users',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('email', sa.String(255), nullable=False, unique=True),
        sa.Column('hashed_password', sa.String(255), nullable=False),
        sa.Column('is_active', sa.Boolean(), nullable=False, server_default='true'),
        sa.Column('is_verified', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('now()')),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('now()')),
        sa.Column('last_login_at', sa.DateTime(timezone=True), nullable=True)
    )
    op.create_index('idx_users_email', 'users', ['email'])
    op.create_index('idx_users_is_active', 'users', ['is_active'])

    # Create user_profiles table
    op.create_table(
        'user_profiles',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False, unique=True),
        sa.Column('experience_level', experience_level_enum, nullable=False),
        sa.Column('professional_role', professional_role_enum, nullable=False),
        sa.Column('role_other', sa.String(100), nullable=True),
        sa.Column('organization', sa.String(255), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('now()')),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('now()')),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE')
    )
    op.create_index('idx_user_profiles_user_id', 'user_profiles', ['user_id'])
    op.create_index('idx_user_profiles_exp_level', 'user_profiles', ['experience_level'])
    op.create_index('idx_user_profiles_prof_role', 'user_profiles', ['professional_role'])

    # Create refresh_tokens table
    op.create_table(
        'refresh_tokens',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('token', sa.String(255), nullable=False, unique=True),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('revoked', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('revoked_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=False, server_default=sa.text('now()')),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE')
    )
    op.create_index('idx_refresh_tokens_user_id', 'refresh_tokens', ['user_id'])
    op.create_index('idx_refresh_tokens_token', 'refresh_tokens', ['token'])
    op.create_index('idx_refresh_tokens_expires_at', 'refresh_tokens', ['expires_at'])

def downgrade():
    op.drop_table('refresh_tokens')
    op.drop_table('user_profiles')
    op.drop_table('users')

    postgresql.ENUM(name='experience_level_enum').drop(op.get_bind())
    postgresql.ENUM(name='professional_role_enum').drop(op.get_bind())
```

---

## Data Access Patterns

### Common Queries

**1. Find user by email**:
```python
user = db.query(User).filter(User.email == email.lower()).first()
```

**2. Get user with profile**:
```python
user = db.query(User).options(joinedload(User.profile)).filter(User.id == user_id).first()
```

**3. Create user with profile (transaction)**:
```python
user = User(email=email.lower(), hashed_password=hashed_pwd)
db.add(user)
db.flush()  # Get user.id

profile = UserProfile(user_id=user.id, experience_level=exp_level, professional_role=role)
db.add(profile)
db.commit()
```

**4. Validate refresh token**:
```python
from datetime import datetime

token_record = db.query(RefreshToken).filter(
    RefreshToken.token == token,
    RefreshToken.revoked == False,
    RefreshToken.expires_at > datetime.utcnow()
).first()
```

**5. Revoke all user tokens**:
```python
db.query(RefreshToken).filter(
    RefreshToken.user_id == user_id,
    RefreshToken.revoked == False
).update({
    "revoked": True,
    "revoked_at": datetime.utcnow()
})
db.commit()
```

---

## State Transitions

### User Account States

```
┌───────────┐
│  Created  │ (is_active=true, is_verified=false)
└─────┬─────┘
      │
      │ (future: email verification)
      ▼
┌───────────┐
│  Active   │ (is_active=true, is_verified=true)
└─────┬─────┘
      │
      │ (admin action / user request)
      ▼
┌───────────┐
│ Suspended │ (is_active=false)
└───────────┘
```

### Refresh Token States

```
┌───────────┐
│  Created  │ (revoked=false, expires_at > now)
└─────┬─────┘
      │
      ├─► (user logs out / token refresh)
      │   ┌───────────┐
      │   │  Revoked  │ (revoked=true)
      │   └───────────┘
      │
      └─► (time passes)
          ┌───────────┐
          │  Expired  │ (expires_at <= now)
          └───────────┘
```

---

## Data Retention & Privacy

**User Data**:
- User accounts retained indefinitely (unless deleted by user/admin)
- Soft delete recommended (set is_active=false) for audit trail
- Hard delete available on explicit user request (GDPR compliance)

**Refresh Tokens**:
- Expired tokens can be purged after 30 days
- Revoked tokens can be purged after 7 days
- Recommended: periodic cleanup job (cron/celery)

**Audit Logs** (future enhancement):
- Consider adding separate audit table for login attempts, password changes, etc.

---

## Performance Considerations

**Indexes**:
- All foreign keys indexed
- Email indexed (frequent lookup)
- Token indexed (frequent validation)
- Experience level and role indexed (future analytics)

**Query Optimization**:
- Use `joinedload()` for user + profile queries (avoid N+1)
- Use `selectinload()` for one-to-many relationships if needed
- Pagination for admin user lists (LIMIT/OFFSET)

**Connection Pooling** (existing):
- SQLAlchemy connection pool configured
- Max connections: 20 (adjust based on load)

---

## Summary

This data model provides:
- ✅ Core authentication (users table)
- ✅ User background collection (user_profiles table)
- ✅ Session management (refresh_tokens table)
- ✅ Revocation capability (for logout/security)
- ✅ Type safety (Pydantic schemas + enums)
- ✅ Database integrity (foreign keys, constraints)
- ✅ Migration support (Alembic)

**Next**: Define API contracts in `/contracts/` directory
