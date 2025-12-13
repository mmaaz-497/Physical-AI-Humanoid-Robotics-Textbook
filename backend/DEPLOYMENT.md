# Backend Deployment Guide (FREE Options)

## Problem Summary
Your FastAPI backend with Qdrant, OpenAI, and PostgreSQL dependencies is **too complex for Vercel's serverless platform**. Vercel is optimized for lightweight functions, not stateful applications with heavy dependencies.

---

## ✅ FREE OPTION 1: Deploy to Render (Recommended)

Render has a generous free tier that's perfect for your app.

### Steps:

1. **Sign up at [Render.com](https://render.com)**
   - Use your GitHub account (easiest)
   - No credit card required for free tier

2. **Create a new Web Service**
   - Click "New" → "Web Service"
   - Connect your GitHub account if not already connected
   - Select this repository: `Physical-AI-Humanoid-Robotics-Textbook`
   - **Important:** Set Root Directory to `backend`

3. **Configure the service:**
   - **Name:** `physical-ai-chatbot` (or any name you like)
   - **Region:** Choose closest to you (e.g., Oregon/Frankfurt/Singapore)
   - **Branch:** `main`
   - **Runtime:** `Python 3`
   - **Build Command:** `pip install -r requirements.txt`
   - **Start Command:** `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - **Instance Type:** `Free`

4. **Set environment variables** (click "Advanced" → "Add Environment Variable"):
   ```
   QDRANT_URL=<your-qdrant-cluster-url>
   QDRANT_API_KEY=<your-qdrant-api-key>
   QDRANT_COLLECTION=physical_ai_textbook
   OPENAI_API_KEY=<your-openai-key>
   OPENAI_MODEL=gpt-4o-mini
   CORS_ORIGINS=https://physical-ai-humanoid-robotics-textb-vert.vercel.app
   ```

   **Important Notes:**
   - Use your actual Qdrant and OpenAI credentials
   - `CORS_ORIGINS` should match your Vercel frontend URL EXACTLY (no trailing slash)
   - You can find your frontend URL in Vercel dashboard

5. **Click "Create Web Service"**
   - Render will start building and deploying
   - This takes 3-5 minutes
   - Watch the logs to ensure no errors

6. **Copy your deployed URL**
   - Once deployed, you'll see a URL like: `https://physical-ai-chatbot.onrender.com`
   - **⚠️ Important:** Free tier apps sleep after 15 minutes of inactivity
   - First request after sleep takes 30-60 seconds to wake up

7. **Update frontend environment variable** in Vercel:
   - Go to [Vercel Dashboard](https://vercel.com/dashboard)
   - Select your project: `physical-ai-humanoid-robotics-textb-vert`
   - Go to **Settings** → **Environment Variables**
   - Click **Add New**
   - Name: `CHATBOT_API_URL`
   - Value: `https://physical-ai-chatbot.onrender.com` (your Render URL)
   - Click **Save**
   - Go to **Deployments** and click **Redeploy** on the latest deployment

8. **Test your chatbot!**

**Free Tier Limitations:**
- App sleeps after 15 mins of inactivity (takes ~30-60s to wake up)
- 750 hours/month (plenty for a chatbot)
- Shared CPU (slower but functional)

---

## ✅ FREE OPTION 2: Deploy to Fly.io

Fly.io also has a free tier with better performance (no sleep).

### Steps:

1. **Install Fly CLI**
   ```bash
   # Windows (PowerShell)
   iwr https://fly.io/install.ps1 -useb | iex

   # Mac/Linux
   curl -L https://fly.io/install.sh | sh
   ```

2. **Sign up and authenticate**
   ```bash
   fly auth signup
   # or if you have an account:
   fly auth login
   ```

3. **Navigate to backend folder**
   ```bash
   cd backend
   ```

4. **Launch your app**
   ```bash
   fly launch
   ```
   - Choose a unique app name: `physical-ai-chatbot-<your-name>`
   - Select region closest to you
   - Don't deploy Postgres database (we use Qdrant)
   - Say **No** to deploy now

5. **Set environment variables**
   ```bash
   fly secrets set QDRANT_URL="<your-qdrant-url>"
   fly secrets set QDRANT_API_KEY="<your-qdrant-key>"
   fly secrets set QDRANT_COLLECTION="physical_ai_textbook"
   fly secrets set OPENAI_API_KEY="<your-openai-key>"
   fly secrets set OPENAI_MODEL="gpt-4o-mini"
   fly secrets set CORS_ORIGINS="https://physical-ai-humanoid-robotics-textb-vert.vercel.app"
   ```

6. **Deploy**
   ```bash
   fly deploy
   ```

7. **Get your URL**
   ```bash
   fly status
   ```
   - URL will be like: `https://physical-ai-chatbot.fly.dev`

8. **Update Vercel** (same as Render step 7 above)

**Free Tier Benefits:**
- No sleep! App stays active
- Better performance than Render
- 3 shared-cpu VMs with 256MB RAM each
- BUT: Requires credit card for verification (won't charge unless you upgrade)

---

## Alternative: Deploy to Render (Manual Method)

If you prefer Render:

1. **Sign up at [Render.com](https://render.com)**

2. **Create a new Web Service**
   - Connect your GitHub repository
   - Select the `backend` directory

3. **Render will use** the `render.yaml` configuration automatically

4. **Set environment variables** (same as Railway above)

5. **Copy the deployed URL** and update `CHATBOT_API_URL` in Vercel

---

## ⚠️ Vercel Deployment (Not Recommended)

Vercel serverless functions have limitations that make them unsuitable for this app:
- **Cold start delays** (5-10 seconds)
- **Package size limits** (Qdrant/OpenAI libraries are large)
- **Execution timeout** (60 seconds max)
- **No persistent connections** (bad for Qdrant client)

If you still want to try Vercel:

1. The configuration files are ready (`vercel.json`, `api/index.py`)
2. Deploy the `backend` folder separately as a Vercel project
3. Set all environment variables in Vercel dashboard
4. Expect slow responses and potential timeouts

**⚠️ Railway or Render are STRONGLY recommended instead.**

---

## Environment Variables Reference

Required for all platforms:
- `QDRANT_URL` - Your Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `QDRANT_COLLECTION` - Collection name (e.g., "physical_ai_textbook")
- `OPENAI_API_KEY` - OpenAI API key
- `CORS_ORIGINS` - Your frontend URL (comma-separated for multiple)

Optional:
- `OPENAI_MODEL` - Default: "gpt-4o-mini"
- `NEON_DATABASE_URL` - If using PostgreSQL for chat logging
- `API_PORT` - Only needed for local development

---

## Testing Your Deployment

1. **Test the health endpoint:**
   ```bash
   curl https://your-backend-url.railway.app/health
   ```
   Should return: `{"status":"ok","version":"1.0.0"}`

2. **Test the API endpoint:**
   ```bash
   curl -X POST https://your-backend-url.railway.app/api/query \
     -H "Content-Type: application/json" \
     -d '{"q": "What is Physical AI?", "top_k": 5}'
   ```

3. **Check chatbot in browser:**
   - Open your Docusaurus site
   - Click the chatbot icon
   - Ask: "What is Physical AI?"
   - Should get a proper response with sources

---

## Troubleshooting

### "CORS error" in browser console
- Make sure `CORS_ORIGINS` includes your exact frontend URL
- Check for trailing slashes - they matter!
- Example: `https://your-site.vercel.app` (no trailing slash)

### "Sorry, I encountered an error"
- Check backend logs in Railway/Render dashboard
- Verify all environment variables are set correctly
- Test the `/health` endpoint to ensure backend is running

### "Connection timeout"
- Qdrant might be unreachable - check `QDRANT_URL` and `QDRANT_API_KEY`
- Verify your Qdrant cluster is running

### Backend deployment fails
- Check build logs in Railway/Render
- Ensure `requirements.txt` is present
- Python version should be 3.11 (specified in `runtime.txt`)

---

## Next Steps

1. Deploy backend to **Railway** (recommended)
2. Get the deployed URL
3. Add `CHATBOT_API_URL` environment variable in **Vercel** (frontend)
4. Redeploy frontend
5. Test the chatbot

**Total deployment time: ~10 minutes**

---

## Need Help?

- Railway Docs: https://docs.railway.app
- Render Docs: https://render.com/docs
- Check your backend logs for specific error messages
