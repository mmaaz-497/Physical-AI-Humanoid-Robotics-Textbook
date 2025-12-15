# Better Auth Setup Guide

Complete setup guide for the authentication system in the Physical AI Humanoid Robotics Textbook project.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Detailed Setup](#detailed-setup)
- [Configuration](#configuration)
- [Database Setup](#database-setup)
- [Running Services](#running-services)
- [Troubleshooting](#troubleshooting)

## Overview

This project uses Better Auth for authentication with the following architecture:

- **Auth Service**: Node.js/TypeScript service (port 3001) handling authentication
- **Backend API**: FastAPI service (port 8000) with JWT validation
- **Frontend**: Docusaurus site (port 3000) with auth UI components
- **Database**: PostgreSQL (Neon) storing users and sessions

## Prerequisites

Install these before starting:

- **Node.js 18+** and npm
- **Python 3.10+** and pip
- **PostgreSQL database** (Neon recommended, or local PostgreSQL)
- **Git** for version control

## Quick Start

### 1. Clone and Install

```bash
# Install root dependencies (Docusaurus)
npm install

# Install auth service dependencies
cd auth-service
npm install
cd ..

# Install backend dependencies
cd backend
pip install -r requirements.txt
cd ..
```

### 2. Configure Environment Variables

#### Auth Service

```bash
cd auth-service
cp .env.example .env
```

Edit `auth-service/.env`:

```env
# Get from Neon.tech or your PostgreSQL provider
DATABASE_URL=postgresql://username:password@host:5432/database

# Generate with: openssl rand -base64 32
BETTER_AUTH_SECRET=<your-secret-key>

# Local development
BETTER_AUTH_URL=http://localhost:3001
PORT=3001
CORS_ORIGINS=http://localhost:3000

# Session config (optional, defaults are fine)
SESSION_EXPIRES_IN=604800  # 7 days
SESSION_UPDATE_AGE=86400    # 24 hours
```

#### Backend API

```bash
cd backend
cp .env.example .env
```

Edit `backend/.env`:

```env
# Qdrant (vector database)
QDRANT_URL=<your-qdrant-url>
QDRANT_API_KEY=<your-qdrant-api-key>
QDRANT_COLLECTION=physical_ai_textbook

# Gemini
GEMINI_API_KEY=<your-gemini-api-key>

# Database (same as auth service)
NEON_DATABASE_URL=postgresql://username:password@host:5432/database

# Auth (must match auth service secret!)
BETTER_AUTH_JWT_SECRET=<same-secret-as-auth-service>

# CORS
CORS_ORIGINS=http://localhost:3000
```

#### Frontend (optional)

Create `.env.local` in project root:

```env
AUTH_SERVICE_URL=http://localhost:3001
CHATBOT_API_URL=http://localhost:8000
```

### 3. Set Up Database

#### Option A: Using provided SQL migration

```bash
cd auth-service

# If you have psql installed
psql $DATABASE_URL -f migrations/0001_initial_schema.sql

# Or connect to your Neon database and run the SQL file contents
```

#### Option B: Using Drizzle Kit

```bash
cd auth-service

# Generate migrations
npm run migrate

# Push to database
npm run migrate:push
```

### 4. Start Services

#### Option 1: Docker Compose (Recommended)

```bash
# Start all services including PostgreSQL
docker-compose up

# Or in detached mode
docker-compose up -d

# View logs
docker-compose logs -f
```

#### Option 2: Manual Start (Development)

Open 3 terminal windows:

**Terminal 1 - Auth Service:**

```bash
cd auth-service
npm run dev
```

**Terminal 2 - Backend API:**

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

**Terminal 3 - Frontend:**

```bash
npm start
```

### 5. Verify Setup

1. Open `http://localhost:3000`
2. Click "Sign Up" in navbar
3. Create a test account
4. Verify you see your name in navbar after signup

## Detailed Setup

### Database Setup (Neon)

1. Go to [neon.tech](https://neon.tech)
2. Create a new project
3. Create a database named `physical_ai_db`
4. Copy the connection string
5. Replace in both `auth-service/.env` and `backend/.env`

**Connection string format:**

```
postgresql://username:password@ep-xxx.region.aws.neon.tech/physical_ai_db?sslmode=require
```

### Generating Secrets

Generate a secure secret for `BETTER_AUTH_SECRET`:

```bash
# On Linux/macOS:
openssl rand -base64 32

# On Windows (PowerShell):
[Convert]::ToBase64String((1..32 | ForEach-Object { Get-Random -Maximum 256 }))

# Example output:
# xK7mP4nR8vW2sE5tY9qL3fH6jU1dC0bA5gT7hN2iO4k=
```

**Important**: Use the same secret in both `auth-service/.env` and `backend/.env`!

### Environment Variables Reference

#### Auth Service (.env)

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `DATABASE_URL` | ✅ | PostgreSQL connection string | `postgresql://...` |
| `BETTER_AUTH_SECRET` | ✅ | Secret key for JWT signing | Generated with openssl |
| `BETTER_AUTH_URL` | ✅ | Auth service public URL | `http://localhost:3001` |
| `PORT` | ❌ | Server port | `3001` |
| `NODE_ENV` | ❌ | Environment | `development` |
| `CORS_ORIGINS` | ✅ | Allowed origins (comma-separated) | `http://localhost:3000` |
| `SESSION_EXPIRES_IN` | ❌ | Session duration in seconds | `604800` (7 days) |
| `SESSION_UPDATE_AGE` | ❌ | Session refresh interval | `86400` (24 hours) |

#### Backend API (.env)

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `QDRANT_URL` | ✅ | Qdrant vector DB URL | `https://xxx.qdrant.io` |
| `QDRANT_API_KEY` | ✅ | Qdrant API key | From Qdrant dashboard |
| `QDRANT_COLLECTION` | ✅ | Collection name | `physical_ai_textbook` |
| `GEMINI_API_KEY` | ✅ | Google Gemini API key | From AI Studio |
| `NEON_DATABASE_URL` | ✅ | PostgreSQL connection | Same as auth service |
| `BETTER_AUTH_JWT_SECRET` | ✅ | JWT secret (must match!) | Same as auth service |
| `CORS_ORIGINS` | ✅ | Allowed origins | `http://localhost:3000` |

### Database Schema

The migrations create these tables:

- **user**: User accounts with custom fields (experience level, role, etc.)
- **session**: Active user sessions with expiration
- **account**: Password hashes and OAuth provider data
- **verification**: Email verification tokens (if enabled)

See `auth-service/README.md` for full schema details.

## Configuration

### Custom User Fields

The signup form collects these custom fields:

- `experienceLevel`: beginner | intermediate | advanced
- `professionalRole`: student | researcher | engineer | hobbyist | other
- `roleOther`: Free text if role is "other"
- `organization`: Optional organization name

These are stored in the `user` table and included in JWT tokens for personalization.

### Session Configuration

Sessions are configured in `auth-service/src/lib/auth.ts`:

- **Expiration**: 7 days (configurable via `SESSION_EXPIRES_IN`)
- **Renewal**: Every 24 hours on activity (configurable via `SESSION_UPDATE_AGE`)
- **Storage**: httpOnly cookies with secure flag in production

### Security Features

The auth service includes:

- **Rate Limiting**:
  - 5 signups per minute
  - 10 signins per minute
  - 30 other requests per minute
- **Security Headers**: X-Frame-Options, CSP, HSTS (production)
- **Password Hashing**: bcrypt (Better Auth default)
- **Request Logging**: All requests logged with IP and user agent
- **Body Size Limit**: 200KB maximum request size

## Running Services

### Development Mode

Start with hot-reload for quick iteration:

```bash
# Auth service
cd auth-service && npm run dev

# Backend
cd backend && uvicorn src.main:app --reload

# Frontend
npm start
```

### Production Mode

Build and run optimized versions:

```bash
# Auth service
cd auth-service
npm run build
npm start

# Backend
cd backend
uvicorn src.main:app --host 0.0.0.0 --port 8000

# Frontend
npm run build
npm run serve
```

### Docker Compose

Full stack with PostgreSQL:

```bash
# Start
docker-compose up -d

# Stop
docker-compose down

# View logs
docker-compose logs -f [service-name]

# Rebuild
docker-compose up --build
```

## Troubleshooting

### Common Issues

#### "DATABASE_URL not set"

**Cause**: Missing or incorrect `.env` file

**Solution**:
1. Verify `auth-service/.env` exists
2. Check `DATABASE_URL` is set and correct
3. Test connection: `psql $DATABASE_URL -c "SELECT 1;"`

#### CORS Errors in Browser

**Cause**: Frontend origin not in `CORS_ORIGINS`

**Solution**:
1. Check `auth-service/.env` has `CORS_ORIGINS=http://localhost:3000`
2. Restart auth service after changing `.env`
3. Clear browser cache and cookies

#### "Invalid token" or "Token expired"

**Cause**: JWT secrets don't match or session expired

**Solution**:
1. Verify `BETTER_AUTH_JWT_SECRET` matches in both `.env` files
2. Clear browser cookies
3. Sign in again

#### Signup form validation errors

**Cause**: Missing required fields or invalid format

**Solution**:
1. Check all required fields have asterisks (*)
2. Verify email format (must contain @)
3. Ensure password is at least 8 characters
4. Check browser console for specific errors

#### Database migration errors

**Cause**: Database not accessible or migrations already applied

**Solution**:
1. Test database connection: `psql $DATABASE_URL -c "SELECT 1;"`
2. Check if tables exist: `\dt` in psql
3. If tables exist, skip migration
4. If starting fresh, drop tables first (see cleanup section)

### Debugging

Enable verbose logging:

**Auth Service:**

```bash
# Set in auth-service/.env
NODE_ENV=development
```

**Backend:**

```bash
# Run with debug logging
cd backend
uvicorn src.main:app --log-level debug
```

**Frontend:**

Open browser DevTools (F12) → Console tab

### Health Checks

Verify each service is running:

```bash
# Auth service
curl http://localhost:3001/health

# Backend
curl http://localhost:8000/health

# Frontend
curl http://localhost:3000
```

### Database Queries

Check database state:

```sql
-- Count users
SELECT COUNT(*) FROM "user";

-- View recent sessions
SELECT "userId", "expiresAt", "ipAddress"
FROM "session"
ORDER BY "expiresAt" DESC
LIMIT 5;

-- Check for specific user
SELECT email, name, experience_level, professional_role
FROM "user"
WHERE email = 'test@example.com';
```

### Cleanup Test Data

Remove test users:

```sql
-- Delete test sessions
DELETE FROM "session"
WHERE "userId" IN (
  SELECT id FROM "user" WHERE email LIKE 'test%@%'
);

-- Delete test accounts
DELETE FROM "account"
WHERE "userId" IN (
  SELECT id FROM "user" WHERE email LIKE 'test%@%'
);

-- Delete test users
DELETE FROM "user" WHERE email LIKE 'test%@%';
```

## Next Steps

After setup:

1. **Test**: Follow `TESTING_GUIDE.md` to verify all features
2. **Customize**: Modify signup questions in `src/pages/auth/signup.tsx`
3. **Deploy**: See `DEPLOYMENT.md` for production deployment
4. **Monitor**: Set up logging and monitoring for production

## Getting Help

- **Auth Service Issues**: Check `auth-service/README.md`
- **API Integration**: See `backend/src/routers/query.py` for examples
- **Frontend**: Review `src/pages/auth/` components
- **Testing**: Use `TESTING_GUIDE.md` for comprehensive tests

## Resources

- [Better Auth Documentation](https://better-auth.com)
- [Drizzle ORM Documentation](https://orm.drizzle.team)
- [FastAPI Documentation](https://fastapi.tiangolo.com)
- [Docusaurus Documentation](https://docusaurus.io)
