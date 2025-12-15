# How to Start the Authentication System

## ✅ All Issues Fixed!

**Latest fixes (just committed):**
- ✅ Fixed Better Auth import path (was using wrong module)
- ✅ Installed correct Better Auth package (v1.4.7)
- ✅ Added dotenv to load environment variables
- ✅ Added @hono/node-server for HTTP server

## Quick Start

### 1. Open 3 Terminals

**Terminal 1 - Auth Service:**
```bash
cd auth-service
npm run dev
```
Wait for: `✅ Auth service listening on http://localhost:3001`

**Terminal 2 - Backend API:**
```bash
cd backend
uvicorn src.main:app --reload --port 8000
```
Wait for: `Uvicorn running on http://127.0.0.1:8000`

**Terminal 3 - Frontend:**
```bash
npm start
```
Wait for: `Docusaurus website is running at http://localhost:3000`

### 2. Test Authentication

1. Open browser: `http://localhost:3000`
2. Click **"Sign Up"** button (top right)
3. Fill out the form:
   - Name: Test User
   - Email: test@example.com
   - Password: TestPassword123
   - Confirm Password: TestPassword123
   - Select experience level: Beginner
   - Select at least one programming language
   - Select at least one interest area
   - Select hardware skill level
   - Select professional role: Student
4. Click **"Create Account"**
5. ✅ You should see your name in the navbar!

### 3. Troubleshooting

**"Port 3001 already in use":**
```bash
# Windows:
netstat -ano | findstr :3001
taskkill /PID <PID_NUMBER> /F

# Linux/Mac:
lsof -ti:3001 | xargs kill -9
```

**"DATABASE_URL not set":**
- Check `auth-service/.env` file exists
- Verify it contains your DATABASE_URL

**CORS errors:**
- Make sure all 3 services are running
- Check browser console (F12) for specific errors

**Auth service won't start:**
```bash
cd auth-service
rm -rf node_modules
npm install --legacy-peer-deps
npm run dev
```

### 4. Verify Everything Works

Test the health endpoints:

```bash
# Auth service
curl http://localhost:3001/health

# Backend
curl http://localhost:8000/health

# Frontend
curl http://localhost:3000
```

All should return `200 OK`.

### 5. What's Been Fixed

✅ Auth service now loads .env file correctly (dotenv package)
✅ Auth service starts HTTP server properly (@hono/node-server)
✅ Database migrations completed
✅ All environment variables configured
✅ Security features enabled (rate limiting, headers, logging)

### 6. Next Steps

Once everything works:
- Test signup/signin/logout flows
- Check mobile responsiveness (resize browser)
- Try using the chatbot as authenticated user
- Review logs in terminal windows

### Common Issues

**Issue**: "Module not found: dotenv"
**Fix**: `cd auth-service && npm install dotenv --legacy-peer-deps`

**Issue**: "@hono/node-server not found"
**Fix**: `cd auth-service && npm install @hono/node-server --legacy-peer-deps`

**Issue**: Backend can't validate JWT
**Fix**: Verify `BETTER_AUTH_JWT_SECRET` matches in both `auth-service/.env` and `backend/.env`

---

**Need Help?** See `SETUP.md` for detailed instructions or `TESTING_GUIDE.md` for test scenarios.
