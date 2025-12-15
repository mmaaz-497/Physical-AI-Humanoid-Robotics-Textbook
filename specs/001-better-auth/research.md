# Research: Better Auth Authentication Implementation

**Feature**: Better Auth Authentication System
**Date**: 2025-12-14
**Branch**: `001-better-auth`

## Executive Summary

After comprehensive research, we've determined that **Better Auth cannot be used with our Python/FastAPI backend** as it is JavaScript/TypeScript-specific and tightly coupled to Node.js runtime. Instead, we will implement a custom FastAPI authentication system using industry-standard libraries that provides equivalent functionality.

## Research Questions & Answers

### 1. Better Auth Compatibility Analysis

**Question**: Can Better Auth be integrated with our Docusaurus + FastAPI stack?

**Answer**: **NO** - Better Auth is not compatible with Python backends.

**Findings**:
- Better Auth is a TypeScript/JavaScript framework requiring Node.js runtime
- No protocol specification exists for cross-language implementation
- Attempting to use Better Auth would require maintaining a separate Next.js service
- This violates our architectural simplicity principle and adds operational overhead

**Source**: Better Auth documentation (better-auth.com), architecture analysis

---

### 2. Recommended Authentication Approach

**Decision**: Custom FastAPI Authentication System

**Rationale**:
1. **Native Integration**: Works directly with existing FastAPI + SQLAlchemy + PostgreSQL stack
2. **Full Control**: Complete control over auth logic and data models
3. **Production-Ready**: Uses battle-tested libraries (Passlib, python-jose)
4. **Feature Parity**: Matches all Better Auth capabilities
5. **Single Stack**: No Node.js microservice needed
6. **Type Safety**: Pydantic models provide TypeScript-like validation

**Alternatives Considered**:

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Better Auth + Next.js Microservice | Official Better Auth support | Adds Node.js dependency, operational complexity, dual backend maintenance | ❌ Rejected |
| FastAPI-Users | Full-featured, popular library | Adds abstraction layer, opinionated structure, learning curve | ⚠️ Considered but rejected |
| Custom FastAPI Implementation | Full control, simple, matches existing patterns | More code to maintain | ✅ **SELECTED** |

---

### 3. Core Technology Stack

**Selected Libraries**:

```python
# Password Hashing & Security
passlib[argon2]==1.7.4          # OWASP-recommended password hashing
python-jose[cryptography]==3.3.0 # JWT token generation/validation

# Database (existing)
sqlalchemy==2.0.36               # ORM
psycopg2-binary==2.9.10         # PostgreSQL adapter
alembic==1.14.1                  # Migrations

# FastAPI (existing)
fastapi==0.115.6                 # Web framework
pydantic==2.10.4                 # Validation
```

**Rationale**:
- **Argon2id**: OWASP 2025 recommended algorithm, memory-hard, GPU-resistant
- **python-jose**: Industry standard for JWT with cryptography support
- **Existing Stack**: Reuses SQLAlchemy, Pydantic, FastAPI patterns

---

### 4. Session Management Architecture

**Decision**: JWT Access Tokens + Database-Backed Refresh Tokens

**Implementation**:

```
┌─────────────────┐
│   Docusaurus    │ (React - in-memory access tokens)
│   Frontend      │
└────────┬────────┘
         │
         │ Authorization: Bearer <JWT>
         │ Cookie: refresh_token=<token> (HTTP-only, Secure)
         │
┌────────▼────────┐
│    FastAPI      │
│                 │
│  Auth Routes:   │
│  POST /auth/signup
│  POST /auth/login
│  POST /auth/logout
│  POST /auth/refresh
│  GET  /auth/me
│                 │
└────────┬────────┘
         │
┌────────▼────────┐
│   PostgreSQL    │
│                 │
│  Tables:        │
│  - users        │
│  - user_profiles│
│  - refresh_tokens
└─────────────────┘
```

**Token Strategy**:

| Token Type | Storage | Lifetime | Purpose | Revocable |
|------------|---------|----------|---------|-----------|
| Access Token (JWT) | Memory (React state) | 30 minutes | API authentication | No (short-lived) |
| Refresh Token | HTTP-only cookie + DB | 7 days | Renew access token | Yes (database) |

**Rationale**:
- **Access tokens in memory**: XSS protection (not in localStorage)
- **Refresh tokens in HTTP-only cookies**: CSRF protection + user convenience
- **Database-backed refresh**: Enables logout, session revocation, security events
- **Cross-origin compatible**: Works with Vercel frontend + separate backend domain

---

### 5. Password Security (OWASP 2025 Standards)

**Decision**: Argon2id with recommended parameters

**Implementation**:
```python
from passlib.context import CryptContext

pwd_context = CryptContext(
    schemes=["argon2"],
    deprecated="auto",
    argon2__memory_cost=65536,      # 64 MB
    argon2__time_cost=3,             # iterations
    argon2__parallelism=4            # threads
)
```

**Password Validation Rules**:
- Minimum 8 characters
- At least one uppercase letter
- At least one lowercase letter
- At least one digit
- At least one special character
- No common passwords (optional: implement check against breach database)

**Rate Limiting**:
- **Login**: 5 attempts per 5 minutes (per IP + email)
- **Signup**: 3 attempts per hour (per IP)
- **Password Reset**: 3 requests per hour (per email)

**Source**: OWASP Password Storage Cheat Sheet 2025

---

### 6. Database Schema Design

**Tables**:

#### users
```python
class User(Base):
    __tablename__ = "users"

    id = UUID (PK)
    email = String (unique, indexed)
    hashed_password = String
    is_active = Boolean (default=True)
    is_verified = Boolean (default=False)
    created_at = DateTime
    updated_at = DateTime
    last_login_at = DateTime (nullable)
```

#### user_profiles
```python
class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = UUID (PK)
    user_id = UUID (FK → users.id, unique)
    experience_level = Enum (beginner, intermediate, advanced)
    professional_role = Enum (student, researcher, engineer, hobbyist, other)
    role_other = String (nullable)
    organization = String (nullable)
    created_at = DateTime
    updated_at = DateTime
```

#### refresh_tokens
```python
class RefreshToken(Base):
    __tablename__ = "refresh_tokens"

    id = UUID (PK)
    user_id = UUID (FK → users.id)
    token = String (unique, indexed)
    expires_at = DateTime
    revoked = Boolean (default=False)
    revoked_at = DateTime (nullable)
    created_at = DateTime
```

**Relationships**:
- User ↔ UserProfile: one-to-one
- User ↔ RefreshToken: one-to-many

---

### 7. Docusaurus Integration Pattern

**Client-Side Auth Flow**:

```typescript
// 1. Auth Context (React Context API)
const AuthContext = React.createContext<AuthContextType>(null);

export function useAuth() {
  return useContext(AuthContext);
}

// 2. Protected Content Component
function ProtectedContent({ children }) {
  const { user, loading } = useAuth();

  if (loading) return <LoadingSpinner />;
  if (!user) return <SignInPrompt />;

  return children;
}

// 3. API Client with Auto-Refresh
async function apiCall(url, options) {
  let response = await fetch(API_BASE + url, {
    ...options,
    credentials: 'include',  // Send cookies
    headers: {
      'Authorization': `Bearer ${getAccessToken()}`,
      ...options.headers
    }
  });

  // Auto-refresh on 401
  if (response.status === 401) {
    await refreshAccessToken();
    response = await fetch(API_BASE + url, options);
  }

  return response;
}
```

**Session Persistence**:
- Access token stored in React state (memory)
- On page load: call `/auth/me` to rehydrate user state
- Refresh token automatically sent via HTTP-only cookie
- If access token expired: call `/auth/refresh` before API requests

---

### 8. Security Best Practices Checklist

#### Password Security
- [x] Argon2id hashing with recommended parameters
- [x] Password strength validation (8+ chars, complexity)
- [x] No password in logs or error messages
- [x] Rate limiting on login/signup endpoints

#### Token Security
- [x] Access tokens: short-lived (30 min), in-memory
- [x] Refresh tokens: HTTP-only, Secure, SameSite cookies
- [x] JWT signed with HS256 (symmetric) or RS256 (asymmetric)
- [x] Refresh token rotation on use
- [x] Database-backed refresh tokens (revocable)

#### CORS & Cookies
- [x] CORS configured for credentials
- [x] Cookie SameSite=None (cross-origin) or Lax (same-origin)
- [x] Secure flag enabled in production (HTTPS)
- [x] Domain and Path properly set

#### HTTPS & Transport
- [x] HTTPS enforced in production
- [x] HSTS header enabled
- [x] TLS 1.2+ only

#### Input Validation
- [x] Email format validation
- [x] Password strength validation
- [x] SQL injection prevention (SQLAlchemy parameterized queries)
- [x] XSS prevention (no innerHTML, proper escaping)

#### Error Handling
- [x] Generic error messages ("Invalid credentials" not "User not found")
- [x] No stack traces in production responses
- [x] Structured logging for security events

---

### 9. API Endpoints Design

**Authentication Routes** (`/auth/*`):

```
POST   /auth/signup
  Request:  { email, password, experience_level, professional_role }
  Response: { user, access_token } + Set-Cookie: refresh_token

POST   /auth/login
  Request:  { email, password }
  Response: { user, access_token } + Set-Cookie: refresh_token

POST   /auth/logout
  Request:  Requires: Authorization header
  Response: 204 No Content + Clear-Cookie: refresh_token

POST   /auth/refresh
  Request:  Cookie: refresh_token
  Response: { access_token }

GET    /auth/me
  Request:  Requires: Authorization header
  Response: { user, profile }
```

---

### 10. Deployment Considerations

**Environment Variables** (`.env`):
```bash
# JWT Configuration
JWT_SECRET_KEY=<random-256-bit-key>
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=7

# Cookie Configuration
COOKIE_DOMAIN=.yourdomain.com
COOKIE_SECURE=true
COOKIE_SAMESITE=none  # For cross-origin (Vercel frontend + separate API domain)

# CORS
CORS_ORIGINS=["https://yourdomain.com", "https://www.yourdomain.com"]
CORS_ALLOW_CREDENTIALS=true

# Database (existing)
DATABASE_URL=postgresql://...
```

**Production Checklist**:
- [ ] HTTPS enabled (SSL certificate)
- [ ] Environment variables secured (not in code)
- [ ] Rate limiting enabled (e.g., slowapi)
- [ ] Database backups configured
- [ ] Logging configured (auth events, errors)
- [ ] Monitoring/alerting for failed auth attempts
- [ ] GDPR compliance for user data storage

---

## Implementation Estimate

| Phase | Effort | Tasks |
|-------|--------|-------|
| **Backend Setup** | 2 days | Database models, migrations, password hashing, JWT utilities |
| **Auth Endpoints** | 2 days | Signup, login, logout, refresh, me endpoints + rate limiting |
| **Frontend Integration** | 2 days | Auth context, API client, protected routes, login/signup UI |
| **Testing** | 1 day | Unit tests (auth logic), integration tests (API), E2E tests (user flow) |
| **Security Hardening** | 1 day | CORS, cookie config, rate limiting, security audit |
| **Documentation** | 0.5 days | API docs, setup guide, deployment notes |
| **TOTAL** | **8.5 days** | |

---

## Risks & Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|-----------|-----------|
| Cross-origin cookie issues | High | Medium | Use SameSite=None with Secure flag; test thoroughly |
| JWT secret leak | Critical | Low | Use environment variables, rotate regularly, monitor |
| Session fixation attacks | High | Low | Regenerate tokens on login, rotate refresh tokens |
| Brute force attacks | Medium | High | Implement rate limiting (5 attempts/5 min) |
| XSS token theft | High | Medium | Store access tokens in memory, never localStorage |

---

## References

1. **Better Auth Documentation**: https://www.better-auth.com/
2. **OWASP Password Storage Cheat Sheet**: https://cheatsheetseries.owasp.org/cheatsheets/Password_Storage_Cheat_Sheet.html
3. **FastAPI Security Tutorial**: https://fastapi.tiangolo.com/tutorial/security/
4. **JWT Best Practices**: https://datatracker.ietf.org/doc/html/rfc8725
5. **MDN HTTP Cookies**: https://developer.mozilla.org/en-US/docs/Web/HTTP/Cookies

---

## Conclusion

We will implement a custom FastAPI authentication system using Argon2id password hashing and JWT-based session management. This approach provides full control, matches our existing architecture, and delivers production-ready security without the complexity of Better Auth's JavaScript-only implementation.

**Next Steps**: Proceed to Phase 1 (Design & Contracts) to create data models and API contracts.
