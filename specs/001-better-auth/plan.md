# Implementation Plan: Better Auth Authentication System

**Branch**: `001-better-auth` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-better-auth/spec.md`

## Summary

Implement authentication using **Better Auth** (TypeScript/Node.js framework) alongside the existing FastAPI backend. The system will provide secure signup/signin functionality with JWT-based session management and user background data collection for future content personalization.

**Key Architecture Decision**: Add a dedicated Node.js/TypeScript authentication service using Better Auth, which will work alongside the existing FastAPI RAG chatbot backend. Both services share the same PostgreSQL database.

**Technical Approach**:
- **Auth Service**: Node.js/TypeScript with Better Auth framework
- **Backend API**: Existing FastAPI (validates tokens from Better Auth)
- **Frontend**: Docusaurus/React with Better Auth React client
- **Database**: Shared PostgreSQL (Neon) - Better Auth tables + existing tables
- **Security**: Better Auth built-in security + JWT token validation

## Technical Context

**Language/Version**:
- Auth Service: TypeScript 5.x, Node.js 20.x
- Backend API: Python 3.11 (existing FastAPI)
- Frontend: JavaScript ES2022/TypeScript, React 18.3.1

**Primary Dependencies**:
- Auth Service: better-auth, drizzle-orm (or prisma), hono (or express)
- Backend API: FastAPI 0.115.6, python-jose (for JWT validation)
- Frontend: React 18.3.1, Docusaurus 3.9.2, @better-auth/react

**Storage**: PostgreSQL 14+ (Neon Serverless) - shared database for both services

**Testing**:
- Auth Service: Vitest (TypeScript testing)
- Backend API: pytest (existing)
- Frontend: Jest/React Testing Library (existing)

**Target Platform**:
- Auth Service: Node.js server (production: same platform as backend)
- Backend API: Linux server (existing)
- Frontend: Web browsers (Chrome, Firefox, Safari, Edge)

**Project Type**: Microservices architecture (auth service + API service + frontend)

**Performance Goals**:
- Authentication response time: <500ms p95
- Token validation: <50ms p95
- Concurrent users: 1000+ simultaneous sessions

**Constraints**:
- No breaking changes to existing documentation content
- Both services must share same PostgreSQL database
- Token validation must work across services
- Mobile-responsive auth UI

**Scale/Scope**:
- Expected users: 10k+ accounts in first year
- Auth service endpoints: 6+ routes (Better Auth standard)
- Backend API: Add token validation middleware
- Frontend components: 5 new React components
- New services: 1 (auth-service)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Docusaurus Frontend                      │
│                  (React - Port 3000 / Vercel)                │
│                                                              │
│  - Better Auth React Client (@better-auth/react)            │
│  - Auth Context & Components                                 │
│  - Protected Routes                                          │
└──────────────┬───────────────────────────┬───────────────────┘
               │                           │
               │ Auth requests             │ API requests (with JWT)
               │ (signup/login/logout)     │
               ▼                           ▼
┌──────────────────────────┐   ┌────────────────────────────┐
│   Better Auth Service    │   │      FastAPI Backend       │
│   (Node.js/TypeScript)   │   │      (Existing - RAG)      │
│      Port 3001           │   │       Port 8000            │
│                          │   │                            │
│  - Better Auth Framework │◄──┤  - Validates JWT tokens   │
│  - Email/Password Auth   │   │  - RAG Chatbot endpoints   │
│  - Session Management    │   │  - Existing services       │
│  - User Profile Fields   │   │                            │
└──────────────┬───────────┘   └────────────┬───────────────┘
               │                            │
               │                            │
               └────────────┬───────────────┘
                            ▼
                  ┌─────────────────────┐
                  │    PostgreSQL       │
                  │  (Neon Serverless)  │
                  │                     │
                  │  Better Auth Tables:│
                  │  - user             │
                  │  - session          │
                  │  - account          │
                  │  - verification     │
                  │                     │
                  │  Existing Tables:   │
                  │  - chat_logs        │
                  │  - book_index       │
                  └─────────────────────┘
```

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Note**: The project constitution file is currently a template placeholder. The following checks are based on standard software development best practices:

### Code Quality Standards
- ✅ **Type Safety**: TypeScript for auth service, Pydantic for FastAPI, TypeScript for frontend
- ✅ **Error Handling**: Better Auth built-in error handling + FastAPI exceptions
- ✅ **Logging**: Structured logging for auth events in both services
- ✅ **Documentation**: Better Auth auto-generated docs + FastAPI OpenAPI

### Testing Requirements
- ✅ **Unit Tests**: Auth service logic, JWT validation, frontend components
- ✅ **Integration Tests**: Auth flow, token validation between services
- ✅ **E2E Tests**: Complete signup/signin/logout user flows

### Security Standards
- ✅ **Password Security**: Better Auth uses bcrypt by default (OWASP compliant)
- ✅ **Token Security**: JWT with configurable expiration, secure cookies
- ✅ **Input Validation**: Better Auth built-in validation + Zod schemas
- ✅ **Rate Limiting**: Better Auth plugin support

### Architecture Principles
- ✅ **Separation of Concerns**: Auth service separate from API service
- ✅ **Microservices**: Independent deployment and scaling
- ✅ **Shared Database**: Single source of truth for user data
- ⚠️ **Service Communication**: Requires JWT validation in FastAPI (documented pattern)

**Status**: ✅ PASSED - All critical checks satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-better-auth/
├── spec.md                    # Feature specification (completed)
├── plan.md                    # This file (implementation plan)
├── research.md                # Better Auth integration research
├── data-model.md              # Better Auth database schema
├── quickstart.md              # Better Auth setup guide
├── contracts/                 # API contracts
│   ├── auth-service.yaml     # Better Auth endpoints
│   └── fastapi-integration.md # Token validation guide
├── checklists/                # Quality validation
│   └── requirements.md        # Spec quality checklist
└── tasks.md                   # Phase 2 output (/sp.tasks - NOT YET CREATED)
```

### Source Code (repository root)

```text
# NEW: Auth Service (Node.js/TypeScript)
auth-service/
├── src/
│   ├── index.ts                  # Entry point (Hono/Express app)
│   ├── lib/
│   │   └── auth.ts              # Better Auth configuration
│   ├── routes/
│   │   └── auth.ts              # Auth routes (if using Hono)
│   ├── db/
│   │   ├── schema.ts            # Drizzle schema (or Prisma schema)
│   │   └── client.ts            # Database client
│   └── middleware/
│       └── cors.ts              # CORS configuration
├── drizzle.config.ts            # Drizzle configuration
├── package.json
├── tsconfig.json
├── .env                         # Auth service environment variables
└── tests/
    └── auth.test.ts             # Auth service tests

# Backend (FastAPI - Existing + Updates)
backend/
├── src/
│   ├── models/
│   │   ├── database.py           # Existing models (ChatLog, BookIndex)
│   │   └── auth_schemas.py       # NEW: JWT payload schemas
│   ├── routers/
│   │   └── query.py              # Existing RAG chatbot endpoint
│   ├── middleware/
│   │   └── auth.py               # NEW: JWT validation middleware
│   ├── utils/
│   │   └── jwt_validator.py      # NEW: Validate Better Auth JWTs
│   ├── config.py                 # UPDATE: Add Better Auth JWT public key
│   └── main.py                   # UPDATE: Add auth middleware
├── requirements.txt              # UPDATE: Add python-jose
└── tests/
    └── test_jwt_validation.py    # NEW: JWT validation tests

# Frontend (Docusaurus - Updates)
src/
├── lib/
│   └── auth-client.ts            # NEW: Better Auth React client
├── components/
│   ├── AuthButton.tsx            # NEW: Login/Logout button
│   └── ProtectedContent.tsx      # NEW: Auth-gated content wrapper
├── contexts/
│   └── AuthContext.tsx           # NEW: React Context for auth state
├── pages/
│   ├── index.js                  # Existing homepage
│   └── auth/
│       ├── login.tsx             # NEW: Login page (Better Auth UI)
│       └── signup.tsx            # NEW: Signup page (Better Auth UI)
├── theme/
│   └── Root.js                   # UPDATE: Wrap with AuthProvider
└── css/
    └── auth.css                  # NEW: Authentication page styles

# Configuration
├── docker-compose.yml            # UPDATE: Add auth-service container
└── .env.example                  # UPDATE: Add auth service vars
```

**Structure Decision**: Microservices architecture selected because Better Auth requires Node.js/TypeScript runtime. Auth service handles authentication independently, while FastAPI backend validates tokens and serves RAG chatbot API. Both services share PostgreSQL database for data consistency.

## Complexity Tracking

> **Justified Complexity**

| Decision | Why Needed | Alternative Rejected Because |
|----------|------------|------------------------------|
| Two backend services | Better Auth requires Node.js runtime; existing FastAPI backend cannot be abandoned | Migrating entire FastAPI RAG chatbot to Node.js would require rewriting working code |
| Shared database | Single source of truth for user data; allows future integration (e.g., personalized chat history) | Separate databases would require complex data synchronization |
| JWT validation in FastAPI | FastAPI endpoints need to verify user identity from Better Auth tokens | Alternative would be to proxy all requests through auth service (performance overhead) |

## Phase 0: Research & Discovery

**Status**: ✅ COMPLETED (Revised for Better Auth)

**Research Questions**:
1. How does Better Auth work with PostgreSQL?
2. What database adapter should we use (Drizzle vs Prisma)?
3. How to add custom user profile fields in Better Auth?
4. How should FastAPI validate Better Auth JWT tokens?
5. How to deploy Node.js auth service alongside FastAPI?

**Key Findings**:

1. **Better Auth with PostgreSQL**:
   - Supports PostgreSQL via Drizzle ORM or Prisma
   - Auto-generates migration files
   - **Recommended**: Drizzle ORM (lighter weight, better TypeScript support)

2. **Custom User Fields**:
   ```typescript
   user: {
     additionalFields: {
       experienceLevel: { type: "string", required: true },
       professionalRole: { type: "string", required: true },
       organization: { type: "string", required: false }
     }
   }
   ```

3. **Token Validation in FastAPI**:
   - Better Auth issues JWT tokens
   - FastAPI validates using `python-jose`
   - Shared JWT secret or public key verification

4. **Session Management**:
   - Better Auth handles sessions automatically
   - Supports both database sessions and JWT
   - Configurable expiration times

5. **Deployment Strategy**:
   - Both services can run on same server (different ports)
   - Docker Compose for local development
   - Production: separate containers or single server with process manager

**Artifacts**: Updated `research.md` with Better Auth architecture

---

## Phase 1: Design & Contracts

**Status**: ✅ IN PROGRESS (Being Updated)

### 1.1 Better Auth Configuration

**Core Setup**:
```typescript
// auth-service/src/lib/auth.ts
import { betterAuth } from "better-auth"
import { drizzleAdapter } from "better-auth/adapters/drizzle"
import { db } from "../db/client"

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "pg", // PostgreSQL
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },
  user: {
    additionalFields: {
      experienceLevel: {
        type: "string",
        required: true,
        input: true, // Accept during signup
      },
      professionalRole: {
        type: "string",
        required: true,
        input: true,
      },
      roleOther: {
        type: "string",
        required: false,
        input: true,
      },
      organization: {
        type: "string",
        required: false,
        input: true,
      },
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update session every 24 hours
  },
  advanced: {
    generateId: () => crypto.randomUUID(), // Use UUIDs
  },
})
```

**Database Schema** (Drizzle):
```typescript
// auth-service/src/db/schema.ts
import { pgTable, text, timestamp, boolean } from "drizzle-orm/pg-core"

export const user = pgTable("user", {
  id: text("id").primaryKey(),
  email: text("email").notNull().unique(),
  emailVerified: boolean("emailVerified").notNull().default(false),
  name: text("name"),
  createdAt: timestamp("createdAt").notNull(),
  updatedAt: timestamp("updatedAt").notNull(),
  // Custom fields
  experienceLevel: text("experience_level").notNull(),
  professionalRole: text("professional_role").notNull(),
  roleOther: text("role_other"),
  organization: text("organization"),
})

export const session = pgTable("session", {
  id: text("id").primaryKey(),
  expiresAt: timestamp("expiresAt").notNull(),
  ipAddress: text("ipAddress"),
  userAgent: text("userAgent"),
  userId: text("userId").notNull().references(() => user.id),
})

export const account = pgTable("account", {
  id: text("id").primaryKey(),
  accountId: text("accountId").notNull(),
  providerId: text("providerId").notNull(),
  userId: text("userId").notNull().references(() => user.id),
  accessToken: text("accessToken"),
  refreshToken: text("refreshToken"),
  idToken: text("idToken"),
  expiresAt: timestamp("expiresAt"),
  password: text("password"),
})

export const verification = pgTable("verification", {
  id: text("id").primaryKey(),
  identifier: text("identifier").notNull(),
  value: text("value").notNull(),
  expiresAt: timestamp("expiresAt").notNull(),
})
```

### 1.2 API Endpoints

**Better Auth Service** (auto-generated by Better Auth):
```
POST   /api/auth/signup/email         # Email/password signup
POST   /api/auth/signin/email         # Email/password signin
POST   /api/auth/signout              # Sign out (destroy session)
GET    /api/auth/session              # Get current session
POST   /api/auth/session/refresh      # Refresh session
GET    /api/auth/user                 # Get current user
```

**FastAPI Integration** (token validation):
```python
# backend/src/middleware/auth.py
from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer
from jose import jwt, JWTError

security = HTTPBearer()

async def get_current_user(credentials = Depends(security)):
    try:
        payload = jwt.decode(
            credentials.credentials,
            settings.BETTER_AUTH_JWT_SECRET,
            algorithms=["HS256"]
        )
        user_id = payload.get("sub")
        if user_id is None:
            raise HTTPException(status_code=401, detail="Invalid token")
        return {"id": user_id, **payload}
    except JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

# Protected endpoint example
@router.post("/api/query")
async def query_rag(
    request: QueryRequest,
    current_user = Depends(get_current_user)
):
    # current_user contains user_id and other claims
    # Can personalize response based on user.experienceLevel, etc.
    ...
```

### 1.3 Frontend Integration

**Better Auth React Client**:
```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react"

export const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === "production"
    ? "https://auth.yourdomain.com"
    : "http://localhost:3001"
})

export const { useSession, signIn, signOut, signUp } = authClient
```

**Signup Page**:
```tsx
// src/pages/auth/signup.tsx
import { authClient } from "@/lib/auth-client"

export default function SignupPage() {
  const handleSignup = async (e) => {
    e.preventDefault()

    const { data, error } = await authClient.signUp.email({
      email: formData.email,
      password: formData.password,
      name: formData.name,
      // Custom fields
      experienceLevel: formData.experienceLevel,
      professionalRole: formData.professionalRole,
      roleOther: formData.roleOther,
      organization: formData.organization,
    })

    if (error) {
      setError(error.message)
    } else {
      router.push("/")
    }
  }

  // Render signup form...
}
```

**Auth Context**:
```tsx
// src/contexts/AuthContext.tsx
import { useSession } from "@/lib/auth-client"

export function AuthProvider({ children }) {
  const { data: session, isPending } = useSession()

  return (
    <AuthContext.Provider value={{ session, loading: isPending }}>
      {children}
    </AuthContext.Provider>
  )
}
```

### 1.4 Deployment Configuration

**Docker Compose** (local development):
```yaml
version: '3.8'

services:
  auth-service:
    build: ./auth-service
    ports:
      - "3001:3001"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/dbname
      - JWT_SECRET=your-secret-key
      - CORS_ORIGINS=http://localhost:3000
    depends_on:
      - db

  backend:
    build: ./backend
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://user:pass@db:5432/dbname
      - BETTER_AUTH_JWT_SECRET=your-secret-key
    depends_on:
      - db

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=dbname
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=pass
    volumes:
      - postgres-data:/var/lib/postgresql/data

volumes:
  postgres-data:
```

---

## Phase 2: Implementation Tasks

**Status**: ⏭️ DEFERRED to `/sp.tasks` command

The next step is to run `/sp.tasks` to generate a detailed, testable task breakdown with:
- Better Auth service setup
- Drizzle ORM configuration
- FastAPI JWT validation
- Frontend integration
- Testing strategy

**Expected tasks.md sections**:
1. Auth Service Setup (Node.js, TypeScript, Better Auth, Drizzle)
2. Database Configuration (shared PostgreSQL, migrations)
3. FastAPI Integration (JWT validation middleware)
4. Frontend Integration (Better Auth React client, Auth Context, pages)
5. Testing (auth service tests, JWT validation tests, E2E tests)
6. Deployment (Docker Compose, production configuration)

---

## Implementation Phases Overview

### Phase 3: Auth Service Implementation
**Duration**: 3-4 days

- Initialize Node.js/TypeScript project
- Install Better Auth and dependencies
- Configure Drizzle ORM with PostgreSQL
- Set up Better Auth with custom user fields
- Configure CORS for Docusaurus frontend
- Create authentication routes
- Add rate limiting (Better Auth plugin)
- Unit and integration tests

**Deliverables**:
- Working auth service at `http://localhost:3001`
- Better Auth endpoints functional
- Database migrations applied
- Tests passing (80%+ coverage)

### Phase 4: FastAPI Integration
**Duration**: 1-2 days

- Add python-jose dependency
- Create JWT validation middleware
- Update existing endpoints with auth middleware
- Test token validation
- Document integration pattern

**Deliverables**:
- FastAPI validates Better Auth tokens
- Protected endpoints require authentication
- JWT validation tests passing

### Phase 5: Frontend Integration
**Duration**: 2-3 days

- Install @better-auth/react
- Create auth client configuration
- Build signup page with custom fields
- Build login page
- Implement Auth Context
- Add auth button to navbar
- Protected content wrapper component
- Mobile-responsive styling

**Deliverables**:
- Functional signup/login/logout UI
- Session persistence across navigation
- Mobile-responsive auth pages
- Better Auth React hooks working

### Phase 6: Testing & Quality Assurance
**Duration**: 1-2 days

- Auth service unit tests (Vitest)
- FastAPI JWT validation tests (pytest)
- Frontend component tests (Jest)
- E2E tests (signup → login → API call → logout)
- Security audit
- Performance testing

**Deliverables**:
- 85%+ overall test coverage
- All E2E flows passing
- Security checklist completed
- Performance benchmarks documented

### Phase 7: Deployment & Documentation
**Duration**: 1 day

- Docker Compose configuration
- Production environment setup
- HTTPS configuration
- CORS for production domains
- Deploy both services
- Monitoring and logging
- User documentation

**Deliverables**:
- Live authentication system
- Both services deployed
- Complete documentation
- Monitoring configured

---

## Technical Decisions

### 1. Better Auth Framework

**Decision**: Use Better Auth as the authentication provider

**Rationale**:
- Project requirement (TypeScript-based)
- Production-ready with built-in security best practices
- Extensive plugin ecosystem
- Active maintenance and community support
- TypeScript-first design

**Trade-offs**:
- Requires Node.js service alongside FastAPI
- Additional deployment complexity
- Learning curve for Better Auth APIs

### 2. Database Adapter: Drizzle ORM

**Decision**: Use Drizzle ORM over Prisma for Better Auth

**Rationale**:
- Lighter weight (smaller bundle size)
- Better TypeScript inference
- SQL-like syntax (easier for developers familiar with SQL)
- Faster migrations
- Better performance (closer to raw SQL)

**Trade-offs**:
- Less mature than Prisma (but stable)
- Smaller ecosystem
- Manual migration generation (though Better Auth simplifies this)

### 3. Service Communication: JWT Tokens

**Decision**: Better Auth issues JWTs, FastAPI validates them

**Rationale**:
- Stateless authentication (no service-to-service calls)
- Standard approach for microservices
- Better Auth handles token generation
- FastAPI only needs to verify signature

**Trade-offs**:
- JWT secret must be shared between services
- Tokens cannot be immediately revoked (use short expiration)

### 4. Shared Database

**Decision**: Both services use same PostgreSQL database

**Rationale**:
- Single source of truth for user data
- Enables future features (personalized chat based on user profile)
- Simpler backup and data management
- No data synchronization needed

**Trade-offs**:
- Both services must coordinate schema changes
- Database becomes single point of failure (mitigated with proper backup)

### 5. Frontend: Better Auth React Client

**Decision**: Use @better-auth/react hooks and components

**Rationale**:
- Official Better Auth React integration
- Type-safe hooks (useSession, signIn, signOut)
- Auto-handles token refresh
- Optimistic UI updates

**Trade-offs**:
- Couples frontend to Better Auth (but that's the requirement)
- Need to learn Better Auth React APIs

---

## Security Considerations

### Better Auth Built-in Security

1. **Password Storage**:
   - ✅ Bcrypt hashing by default
   - ✅ Configurable rounds (default: 10)
   - ✅ No passwords in logs

2. **Token Security**:
   - ✅ JWT tokens with configurable expiration
   - ✅ Secure session cookies (httpOnly, secure, sameSite)
   - ✅ CSRF protection via cookies

3. **Session Security**:
   - ✅ Database-backed sessions (revocable)
   - ✅ Session renewal on activity
   - ✅ IP and user agent tracking

### Additional Security Measures

1. **CORS Configuration**:
   ```typescript
   // auth-service/src/middleware/cors.ts
   export const corsMiddleware = {
     origin: ["http://localhost:3000", "https://yourdomain.com"],
     credentials: true,
   }
   ```

2. **Rate Limiting**:
   ```typescript
   // Using better-auth rate limit plugin
   import { rateLimit } from "better-auth/plugins"

   export const auth = betterAuth({
     plugins: [
       rateLimit({
         window: 60, // 1 minute
         max: 5, // 5 requests per minute
       }),
     ],
   })
   ```

3. **FastAPI Token Validation**:
   ```python
   # Validate signature + expiration
   payload = jwt.decode(
       token,
       settings.BETTER_AUTH_JWT_SECRET,
       algorithms=["HS256"],
       options={"verify_exp": True}
   )
   ```

---

## Performance Optimization

### Auth Service

1. **Caching**:
   - Session lookups cached in Redis (optional)
   - JWT validation is stateless (no DB lookup)

2. **Connection Pooling**:
   - Drizzle connection pool configured
   - Max connections: 20

3. **Response Times**:
   - Target: <200ms for auth operations
   - JWT generation: <50ms

### FastAPI

1. **JWT Validation**:
   - Stateless (no database call)
   - Cache public key in memory
   - Validation time: <10ms

2. **Middleware Optimization**:
   - Only validate tokens on protected routes
   - Skip validation for public endpoints

---

## Deployment Strategy

### Environment Configuration

**Auth Service** (`.env`):
```env
# Development
NODE_ENV=development
PORT=3001
DATABASE_URL=postgresql://user:pass@localhost:5432/dbname
JWT_SECRET=your-256-bit-secret-key
BETTER_AUTH_URL=http://localhost:3001
CORS_ORIGINS=http://localhost:3000

# Production
NODE_ENV=production
PORT=3001
DATABASE_URL=postgresql://user:pass@db.neon.tech:5432/dbname?sslmode=require
JWT_SECRET=<production-secret>
BETTER_AUTH_URL=https://auth.yourdomain.com
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com
```

**FastAPI** (`.env`):
```env
# Add to existing .env
BETTER_AUTH_JWT_SECRET=<same-as-auth-service>
AUTH_SERVICE_URL=http://localhost:3001  # For service-to-service calls (if needed)
```

### Deployment Options

**Option 1: Single Server** (Development/Small Scale):
```bash
# Run both services on same server
pm2 start auth-service/dist/index.js --name auth
uvicorn backend.src.main:app --host 0.0.0.0 --port 8000
```

**Option 2: Docker Compose** (Recommended):
```bash
docker-compose up -d
```

**Option 3: Separate Containers** (Production):
- Auth service: Deploy to same platform as backend (e.g., Railway, Render)
- FastAPI: Existing deployment (update with JWT validation)
- Use subdomain: `auth.yourdomain.com` and `api.yourdomain.com`

### Deployment Checklist

- [ ] Generate secure JWT_SECRET (256-bit random)
- [ ] Configure DATABASE_URL for production (Neon PostgreSQL)
- [ ] Run Drizzle migrations in production
- [ ] Enable HTTPS for auth service
- [ ] Configure CORS for production domains
- [ ] Update FastAPI with BETTER_AUTH_JWT_SECRET
- [ ] Test authentication flow end-to-end
- [ ] Configure logging and monitoring
- [ ] Set up alerting for auth failures
- [ ] Document deployment process

---

## Monitoring & Observability

### Auth Service Metrics

- Signup rate (per day, per hour)
- Login success/failure rate
- Session creation/revocation rate
- Token refresh rate
- Error rates by endpoint

### FastAPI Metrics

- Token validation success/failure rate
- Protected endpoint access patterns
- User activity by experience level / role

### Logging

**Auth Service** (Better Auth auto-logs):
```typescript
// Better Auth logs authentication events
// Configure custom logger if needed
import { logger } from "better-auth/plugins"

export const auth = betterAuth({
  plugins: [
    logger({
      level: "info",
      destination: "./logs/auth.log",
    }),
  ],
})
```

**FastAPI** (existing + auth events):
```python
import logging

logger = logging.getLogger("auth")

@router.post("/api/query")
async def query_rag(current_user = Depends(get_current_user)):
    logger.info(f"User {current_user['id']} queried RAG", extra={
        "user_id": current_user["id"],
        "experience_level": current_user.get("experienceLevel"),
    })
```

---

## Testing Strategy

### Auth Service Tests

```typescript
// auth-service/tests/auth.test.ts
import { describe, it, expect } from "vitest"
import { auth } from "../src/lib/auth"

describe("Authentication", () => {
  it("should create user with custom fields", async () => {
    const user = await auth.api.signUpEmail({
      body: {
        email: "test@example.com",
        password: "SecurePassword123!",
        name: "Test User",
        experienceLevel: "intermediate",
        professionalRole: "student",
      },
    })

    expect(user.data).toBeDefined()
    expect(user.data?.experienceLevel).toBe("intermediate")
  })

  it("should reject weak passwords", async () => {
    const result = await auth.api.signUpEmail({
      body: {
        email: "test@example.com",
        password: "weak",
        name: "Test",
        experienceLevel: "beginner",
        professionalRole: "student",
      },
    })

    expect(result.error).toBeDefined()
  })
})
```

### FastAPI Integration Tests

```python
# backend/tests/test_jwt_validation.py
import pytest
from jose import jwt
from datetime import datetime, timedelta

def test_valid_jwt_token(client):
    # Create valid JWT token
    payload = {
        "sub": "user123",
        "email": "test@example.com",
        "exp": datetime.utcnow() + timedelta(hours=1)
    }
    token = jwt.encode(payload, settings.BETTER_AUTH_JWT_SECRET, algorithm="HS256")

    response = client.post(
        "/api/query",
        headers={"Authorization": f"Bearer {token}"},
        json={"query": "test"}
    )

    assert response.status_code == 200

def test_expired_jwt_token(client):
    payload = {
        "sub": "user123",
        "exp": datetime.utcnow() - timedelta(hours=1)  # Expired
    }
    token = jwt.encode(payload, settings.BETTER_AUTH_JWT_SECRET, algorithm="HS256")

    response = client.post(
        "/api/query",
        headers={"Authorization": f"Bearer {token}"},
        json={"query": "test"}
    )

    assert response.status_code == 401
```

### E2E Tests

```typescript
// e2e/auth-flow.spec.ts
import { test, expect } from "@playwright/test"

test("complete auth flow", async ({ page }) => {
  // Signup
  await page.goto("http://localhost:3000/auth/signup")
  await page.fill('input[name="email"]', "test@example.com")
  await page.fill('input[name="password"]', "SecurePassword123!")
  await page.selectOption('select[name="experienceLevel"]', "intermediate")
  await page.selectOption('select[name="professionalRole"]', "student")
  await page.click('button[type="submit"]')

  // Should redirect to homepage and show user email
  await expect(page).toHaveURL("http://localhost:3000/")
  await expect(page.locator("text=test@example.com")).toBeVisible()

  // Logout
  await page.click("text=Logout")
  await expect(page.locator("text=Sign In")).toBeVisible()

  // Login
  await page.goto("http://localhost:3000/auth/login")
  await page.fill('input[name="email"]', "test@example.com")
  await page.fill('input[name="password"]', "SecurePassword123!")
  await page.click('button[type="submit"]')

  await expect(page).toHaveURL("http://localhost:3000/")
  await expect(page.locator("text=test@example.com")).toBeVisible()
})
```

---

## Risk Management

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| JWT secret leak | Low | Critical | Use environment variables, rotate regularly, monitor logs |
| Service downtime (auth) | Medium | High | Health checks, auto-restart, load balancing |
| Database connection issues | Low | High | Connection pooling, retry logic, fallback handling |
| CORS misconfiguration | Medium | Medium | Explicit origin whitelist, test with production domains |
| Token validation failure | Low | High | Comprehensive tests, error logging, fallback to re-login |
| Session fixation | Low | Medium | Better Auth handles session regeneration automatically |

---

## Success Criteria

**Functional Requirements** (from spec.md):
- ✅ Users can sign up with email/password + background questions
- ✅ Users can sign in with existing credentials
- ✅ Users can log out and terminate sessions
- ✅ Sessions persist across page navigation
- ✅ All auth pages are mobile-responsive
- ✅ Background data stored for personalization

**Technical Requirements**:
- ✅ Better Auth framework integrated
- ✅ Node.js/TypeScript auth service operational
- ✅ FastAPI validates Better Auth tokens
- ✅ Shared PostgreSQL database
- ✅ Production-ready security

**Non-Functional Requirements**:
- ✅ Authentication response time <500ms (p95)
- ✅ Token validation <50ms
- ✅ Session security: HTTP-only secure cookies
- ✅ Zero breaking changes to existing content
- ✅ 85%+ test coverage

---

## Next Steps

1. **Run `/sp.tasks`**: Generate detailed task breakdown with acceptance criteria and test cases
2. **Implement Auth Service**: Node.js/TypeScript + Better Auth + Drizzle (3-4 days)
3. **Integrate FastAPI**: JWT validation middleware (1-2 days)
4. **Implement Frontend**: Better Auth React client, pages, components (2-3 days)
5. **Testing**: Unit, integration, E2E tests (1-2 days)
6. **Deploy**: Docker Compose, production configuration (1 day)

**Total Estimated Effort**: 8-12 days

---

## References

- **Specification**: [spec.md](./spec.md)
- **Better Auth Documentation**: https://www.better-auth.com/docs
- **Drizzle ORM**: https://orm.drizzle.team/
- **Better Auth React**: https://www.better-auth.com/docs/integrations/react
- **OpenAPI Docs** (FastAPI): http://localhost:8000/docs
- **Auth Service** (when running): http://localhost:3001

---

**Document Status**: ✅ UPDATED for Better Auth - Ready for `/sp.tasks` command
