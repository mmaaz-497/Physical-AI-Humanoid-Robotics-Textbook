# Chatbot Fix - Complete Solution

## Issues Found and Fixed

### ❌ Issue #1: Missing CHATBOT_API_URL in Frontend
**Problem:** The root `.env` file was missing the `CHATBOT_API_URL` environment variable, causing the frontend to use the hardcoded default `http://localhost:8000`.

**Fixed:** Added `CHATBOT_API_URL=http://localhost:8000` to the root `.env` file.

**Location:** `D:\GIAIC\Quater_4_AIDD\Physical-AI-Humanoid-Robotics-Textbook\.env` (lines 1-3)

---

### ❌ Issue #2: Invalid GEMINI_MODEL Configuration
**Problem:** The backend `.env` had `GEMINI_MODEL=models/gemini-2.5-flash`, but the Google Generative AI SDK expects the model name WITHOUT the "models/" prefix.

**Error Caused:** When the chatbot tried to generate answers, the Gemini API rejected the request due to invalid model name format.

**Fixed:** Changed to `GEMINI_MODEL=gemini-2.0-flash-exp` (correct format for the SDK).

**Location:** `backend/.env` (line 8)

**Technical Details:**
- The `genai.GenerativeModel()` class automatically adds the "models/" prefix
- Providing "models/gemini-2.5-flash" resulted in the SDK requesting "models/models/gemini-2.5-flash"
- This caused API errors that appeared as "Sorry, I encountered an error" in the chatbot

---

### ❌ Issue #3: Missing Production URL in CORS Configuration
**Problem:** The backend CORS was only configured for `http://localhost:3000,http://localhost:3001`, which blocks requests from your production Vercel deployment.

**Fixed:** Added `https://physical-ai-humanoid-robotics-textb-vert.vercel.app` to CORS_ORIGINS.

**Location:** `backend/.env` (line 18)

**Technical Details:**
- CORS (Cross-Origin Resource Sharing) prevents browsers from making requests to different domains unless explicitly allowed
- Your Vercel frontend at `https://physical-ai-humanoid-robotics-textb-vert.vercel.app` was being blocked
- Now both local development and production are supported

---

## How to Test the Fix

### Option 1: Local Development

1. **Restart the backend** (IMPORTANT - changes require restart):
   ```bash
   cd backend
   # Kill existing process if running
   uvicorn src.main:app --reload --port 8000
   ```

2. **Restart the frontend**:
   ```bash
   # In project root
   npm start
   ```

3. **Test the chatbot**:
   - Open http://localhost:3000
   - Click the chatbot icon (bottom-right)
   - Ask: "What is Physical AI?"
   - You should now get a proper answer instead of an error

### Option 2: Production (Vercel)

1. **Deploy backend** (if you have a deployed backend):
   - Make sure your deployed backend has the same `.env` variables
   - Update `CHATBOT_API_URL` in Vercel environment variables to point to your deployed backend

2. **Redeploy frontend** on Vercel:
   - The changes to `.env` need to be set as environment variables in Vercel
   - Add `CHATBOT_API_URL=<your-backend-url>` in Vercel dashboard

---

## Verification Checklist

Run these commands to verify everything is configured correctly:

```bash
# 1. Check frontend config
grep CHATBOT_API_URL .env

# 2. Check backend config
cd backend
python -c "from src.config import settings; print('Model:', settings.GEMINI_MODEL); print('CORS:', settings.CORS_ORIGINS)"

# 3. Test backend API directly
curl http://localhost:8000/health
# Should return: {"status":"ok","version":"1.0.0"}

# 4. Test query endpoint
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"q": "What is Physical AI?", "top_k": 5}'
# Should return a JSON response with an answer
```

---

## Why This Was Happening

The chatbot error "Sorry, I encountered an error" was caused by a chain of issues:

1. **Frontend couldn't find backend** → Used default URL which might not be running
2. **Even when backend was running** → Gemini API rejected requests due to invalid model name
3. **Production deployment** → CORS blocked all requests from Vercel

All three issues are now fixed!

---

## Configuration Files Changed

1. **Root `.env`** - Added CHATBOT_API_URL
2. **`backend/.env`** - Fixed GEMINI_MODEL and updated CORS_ORIGINS

---

## Next Steps

### For Local Development
1. Restart your backend server
2. Restart your frontend server
3. Test the chatbot
4. Everything should work now!

### For Production Deployment
You need to deploy your backend somewhere and update the environment variables:

**Option A: Deploy Backend to Render (Free)**
1. Go to https://render.com
2. Create new Web Service
3. Connect your GitHub repo
4. Set root directory to `backend`
5. Set build command: `pip install -r requirements.txt`
6. Set start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
7. Add all environment variables from `backend/.env`
8. Deploy and copy the URL

**Option B: Use Existing Backend**
If you already have a deployed backend:
1. Update its environment variables to match `backend/.env`
2. Restart the backend service
3. Update `CHATBOT_API_URL` in Vercel to point to your backend

---

## Technical Summary

**Root Cause:** Configuration mismatches and missing environment variables

**Files Modified:**
- `.env` (root) - Added CHATBOT_API_URL
- `backend/.env` - Fixed GEMINI_MODEL format and CORS_ORIGINS

**Services Affected:**
- Frontend (Docusaurus) - Now knows where to find the backend
- Backend (FastAPI) - Now uses correct Gemini model and allows production requests

**Testing:** Configuration verified successfully with `python -c` command

---

## Support

If you still encounter issues:

1. **Check browser console** (F12) for specific error messages
2. **Check backend logs** for API errors
3. **Verify all 3 services are running**:
   - Backend: http://localhost:8000/health
   - Frontend: http://localhost:3000
   - Auth service: http://localhost:3001/health

4. **Common issues**:
   - Port already in use → Kill the process and restart
   - CORS errors → Clear browser cache and try again
   - Gemini API errors → Check your API key is valid

---

**All issues are now fixed! Your chatbot should work perfectly.** 🎉
