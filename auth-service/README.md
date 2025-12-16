# Authentication Service

Better Auth authentication microservice for the Physical AI Humanoid Robotics Textbook.

## Features

- Email/password authentication
- Custom user profile fields (experience level, professional role)
- Secure session management (7-day expiration, 24-hour renewal)
- JWT token generation for FastAPI integration
- PostgreSQL database with Drizzle ORM

## Prerequisites

- Node.js 18+ and npm
- PostgreSQL database (Neon, Supabase, or local)
- Better Auth secret key

## Setup

### 1. Install Dependencies

```bash
cd auth-service
npm install
```

### 2. Configure Environment Variables

Copy `.env.example` to `.env` and fill in your values:

```bash
cp .env.example .env
```

Required environment variables:

```env
# Database URL (Neon, Supabase, or local PostgreSQL)
DATABASE_URL=postgresql://username:password@host:5432/database_name

# Better Auth secret (generate with: openssl rand -base64 32)
BETTER_AUTH_SECRET=your-256-bit-secret-key-change-this-in-production

# Auth service URL
BETTER_AUTH_URL=http://localhost:3001

# Server port
PORT=3001

# CORS origins (comma-separated)
CORS_ORIGINS=http://localhost:3000

# Session configuration
SESSION_EXPIRES_IN=604800  # 7 days in seconds
SESSION_UPDATE_AGE=86400    # 24 hours in seconds
```

### 3. Set Up Database

#### Option A: Run Migration SQL Manually

If you have direct access to your PostgreSQL database:

```bash
psql $DATABASE_URL -f migrations/0001_initial_schema.sql
```

#### Option B: Use Drizzle Kit (Recommended)

Generate and push migrations using Drizzle Kit:

```bash
# Generate migration files
npm run migrate

# Push to database
npm run migrate:push
```

### 4. Start Development Server

```bash
npm run dev
```

The auth service will start on `http://localhost:3001`.

### 5. Verify Setup

Test the health check endpoint:

```bash
curl http://localhost:3001/health
```

Expected response:

```json
{
  "status": "ok",
  "service": "auth-service",
  "timestamp": "2025-12-15T..."
}
```

## API Endpoints

All Better Auth endpoints are available under `/api/auth/`:

### Signup

```bash
POST /api/auth/signup/email
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword123",
  "name": "John Doe",
  "experienceLevel": "beginner",
  "professionalRole": "student",
  "organization": "MIT"
}
```

### Signin

```bash
POST /api/auth/signin/email
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword123"
}
```

### Signout

```bash
POST /api/auth/signout
Cookie: session=...
```

### Get Session

```bash
GET /api/auth/session
Cookie: session=...
```

## Database Schema

### User Table

- `id` (TEXT, PRIMARY KEY)
- `email` (TEXT, UNIQUE, NOT NULL)
- `emailVerified` (BOOLEAN, DEFAULT false)
- `name` (TEXT)
- `createdAt` (TIMESTAMP, NOT NULL)
- `updatedAt` (TIMESTAMP, NOT NULL)
- `image` (TEXT)
- **Custom Fields:**
  - `experience_level` (TEXT, NOT NULL) - 'beginner' | 'intermediate' | 'advanced'
  - `professional_role` (TEXT, NOT NULL) - 'student' | 'researcher' | 'engineer' | 'hobbyist' | 'other'
  - `role_other` (TEXT)
  - `organization` (TEXT)

### Session Table

- `id` (TEXT, PRIMARY KEY)
- `expiresAt` (TIMESTAMP, NOT NULL)
- `ipAddress` (TEXT)
- `userAgent` (TEXT)
- `userId` (TEXT, FOREIGN KEY → user.id)
- `activeOrganizationId` (TEXT)
- `impersonatedBy` (TEXT)

### Account Table

- `id` (TEXT, PRIMARY KEY)
- `accountId` (TEXT, NOT NULL)
- `providerId` (TEXT, NOT NULL)
- `userId` (TEXT, FOREIGN KEY → user.id)
- `accessToken` (TEXT)
- `refreshToken` (TEXT)
- `idToken` (TEXT)
- `expiresAt` (TIMESTAMP)
- `password` (TEXT) - Hashed with bcrypt

### Verification Table

- `id` (TEXT, PRIMARY KEY)
- `identifier` (TEXT, NOT NULL)
- `value` (TEXT, NOT NULL)
- `expiresAt` (TIMESTAMP, NOT NULL)
- `createdAt` (TIMESTAMP)
- `updatedAt` (TIMESTAMP)

## Development

### Run Tests

```bash
npm test
```

### Build for Production

```bash
npm run build
npm start
```

## Security Features

- Password hashing with bcrypt (Better Auth default)
- httpOnly and secure cookies in production
- CORS protection with configurable origins
- Session expiration and automatic renewal
- JWT signing with secret key

## Troubleshooting

### "DATABASE_URL environment variable is not set"

Make sure your `.env` file exists and contains a valid `DATABASE_URL`.

### "Invalid token: expired"

Session tokens expire after 7 days. User needs to sign in again.

### CORS errors

Add your frontend URL to the `CORS_ORIGINS` environment variable:

```env
CORS_ORIGINS=http://localhost:3000,https://your-frontend.vercel.app
```

### Database connection errors

Verify your `DATABASE_URL` is correct and the database is accessible:

```bash
psql $DATABASE_URL -c "SELECT 1;"
```

## Production Deployment

See the main project's `DEPLOYMENT.md` for production deployment instructions.

## License

MIT
