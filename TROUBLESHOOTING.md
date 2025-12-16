# Troubleshooting Guide

## Issue 1: Chatbot Error "❌ Sorry, I encountered an error"

### Problem
The chatbot shows an error message even after deploying the backend to Render.

### Root Cause
The chatbot widget's API URL is not configured correctly for the deployed backend.

### How the Chatbot API URL Works

1. **Frontend Configuration** (`docusaurus.config.js:24`):
   ```javascript
   customFields: {
     chatbotApiUrl: process.env.CHATBOT_API_URL || 'http://localhost:8000',
   }
   ```

2. **Widget Initialization** (`src/theme/Root.tsx:26-28`):
   ```typescript
   const apiUrl =
     (siteConfig.customFields?.chatbotApiUrl as string) || 'http://localhost:8000';
   script.setAttribute('data-api-url', apiUrl);
   ```

3. **Widget Usage** (`static/widget.js:132`):
   ```javascript
   const response = await fetch(`${this.apiUrl}/api/query`, { ... });
   ```

### Solution

#### For Local Development
The chatbot defaults to `http://localhost:8000` which works when running the backend locally.

#### For Vercel Deployment

Set the `CHATBOT_API_URL` environment variable in Vercel:

1. Go to your Vercel project dashboard
2. Navigate to **Settings** → **Environment Variables**
3. Add a new variable:
   - **Key**: `CHATBOT_API_URL`
   - **Value**: `https://your-backend-app.onrender.com` (your Render backend URL)
   - **Environment**: Production (and Preview if needed)
4. Redeploy your Vercel app

#### For GitHub Pages Deployment

Create a `.env` file in the project root (DO NOT commit it):
```bash
CHATBOT_API_URL=https://your-backend-app.onrender.com
```

Or set it during build:
```bash
CHATBOT_API_URL=https://your-backend-app.onrender.com npm run build
```

### Verification Steps

1. **Check the widget initialization** - Open browser console and look for:
   ```
   Chatbot widget script injected with API URL: https://your-backend-app.onrender.com
   ```

2. **Test the backend endpoint** directly:
   ```bash
   curl -X POST https://your-backend-app.onrender.com/api/query \
     -H "Content-Type: application/json" \
     -d '{"q": "What is Physical AI?", "top_k": 5}'
   ```

3. **Check for CORS errors** in the browser console. If you see CORS errors, ensure your backend's `CORS_ORIGINS` environment variable includes your frontend URL.

### Backend CORS Configuration

In your Render backend, set the `CORS_ORIGINS` environment variable:
```
CORS_ORIGINS=https://your-vercel-app.vercel.app,https://your-github-pages-url
```

The backend reads this in `backend/src/config.py` and configures CORS accordingly.

---

## Issue 2: Better Auth Routes Returning 404

### Problem
Auth service endpoints like `/api/auth/sign-up/email` and `/api/auth/sign-in/email` return 404 errors.

### Root Causes Identified

1. **Incorrect Hono route pattern**: Used `/api/auth/**` instead of `/api/auth/*`
2. **Database tables may not exist**: Better Auth requires specific database tables

### Solutions Applied

#### 1. Fixed Hono Route Pattern
Changed `auth-service/src/index.ts:31` from:
```typescript
app.on(['POST', 'GET'], '/api/auth/**', (c) => {
  return auth.handler(c.req.raw);
});
```

To:
```typescript
app.on(['POST', 'GET'], '/api/auth/*', (c) => {
  return auth.handler(c.req.raw);
});
```

**Why**: Hono uses single asterisk (`*`) for wildcard routes, not double (`**`).

#### 2. Database Schema Migration Required

The Better Auth tables need to be created in your Neon database. Run:

```bash
cd auth-service
npm run migrate:push
```

When prompted, select **"Yes, I want to execute all statements"**.

This creates the following tables:
- `user` (with custom fields: experienceLevel, professionalRole, organization)
- `session`
- `account`
- `verification`

### Better Auth Endpoint Examples

Once fixed, these endpoints should work:

**Sign Up**:
```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "securepass123",
    "name": "John Doe",
    "experienceLevel": "beginner",
    "professionalRole": "student"
  }'
```

**Sign In**:
```bash
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "securepass123"
  }'
```

**Get Session**:
```bash
curl -X GET http://localhost:3001/api/auth/session \
  -H "Cookie: better-auth.session_token=<your-session-token>"
```

---

## Quick Diagnosis Checklist

### Chatbot Not Working
- [ ] Check browser console for the chatbot API URL log message
- [ ] Verify `CHATBOT_API_URL` is set in Vercel environment variables
- [ ] Test backend `/api/query` endpoint directly with curl
- [ ] Check for CORS errors in browser console
- [ ] Verify backend is deployed and running on Render
- [ ] Check backend logs on Render for errors

### Auth Service 404 Errors
- [ ] Verify auth service is running (`curl http://localhost:3001/health`)
- [ ] Check database schema is migrated (`npm run migrate:push`)
- [ ] Test a simple auth endpoint (`/api/auth/session`)
- [ ] Check auth service logs for database connection errors
- [ ] Verify `.env` file has `DATABASE_URL` and `BETTER_AUTH_SECRET`

---

## Common Environment Variables

### Frontend (Vercel/Build)
```bash
CHATBOT_API_URL=https://your-backend.onrender.com
```

### Backend (Render)
```bash
# Backend API
PORT=8000
HOST=0.0.0.0
CORS_ORIGINS=https://your-frontend.vercel.app

# Gemini API
GEMINI_API_KEY=your-gemini-api-key
GEMINI_MODEL=gemini-2.0-flash-exp

# Qdrant
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-key

# Neon Database (optional)
NEON_DATABASE_URL=your-neon-connection-string
```

### Auth Service (Render - separate service)
```bash
# Database
DATABASE_URL=postgresql://...your-neon-url...

# Auth Configuration
BETTER_AUTH_SECRET=your-secret-key-here
BETTER_AUTH_URL=https://your-auth-service.onrender.com
PORT=3001

# CORS (frontend URL)
CORS_ORIGINS=https://your-frontend.vercel.app

# Session
SESSION_EXPIRES_IN=604800
SESSION_UPDATE_AGE=86400
```

---

## Next Steps

1. **For the chatbot issue**: Set `CHATBOT_API_URL` in Vercel and redeploy
2. **For the auth service**: Run database migration and restart the service
3. **Test both systems** end-to-end after fixes

If you continue to experience issues, check the respective service logs:
- **Backend logs**: Render dashboard → Your backend service → Logs
- **Auth service logs**: Render dashboard → Your auth service → Logs
- **Frontend**: Browser console (F12)
