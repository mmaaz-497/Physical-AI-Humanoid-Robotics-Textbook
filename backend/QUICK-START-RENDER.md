# 🚀 Quick Start: Deploy Backend to Render (FREE)

## Total Time: ~10 minutes

---

## Step 1: Sign Up for Render (2 minutes)

1. Go to **[render.com](https://render.com)**
2. Click **"Get Started for Free"**
3. Sign up with **GitHub** (easiest option)
4. Authorize Render to access your GitHub
5. ✅ No credit card required!

---

## Step 2: Create Web Service (3 minutes)

1. Click **"New +"** button (top right)
2. Select **"Web Service"**
3. Find and select your repository: `Physical-AI-Humanoid-Robotics-Textbook`
   - If you don't see it, click "Configure account" to grant access
4. Click **"Connect"**

---

## Step 3: Configure Service (2 minutes)

Fill in these settings:

| Field | Value |
|-------|-------|
| **Name** | `physical-ai-chatbot` (or any name) |
| **Region** | Choose closest to you |
| **Root Directory** | `backend` ⚠️ IMPORTANT! |
| **Runtime** | `Python 3` |
| **Build Command** | `pip install -r requirements.txt` |
| **Start Command** | `uvicorn src.main:app --host 0.0.0.0 --port $PORT` |
| **Instance Type** | **Free** |

---

## Step 4: Add Environment Variables (2 minutes)

Click **"Advanced"** then add these variables one by one:

### Required Variables:

1. **QDRANT_URL**
   - Value: Your Qdrant cluster URL (e.g., `https://xxx.qdrant.io`)

2. **QDRANT_API_KEY**
   - Value: Your Qdrant API key

3. **QDRANT_COLLECTION**
   - Value: `physical_ai_textbook`

4. **GEMINI_API_KEY**
   - Value: Your Gemini API key

5. **GEMINI_MODEL**
   - Value: `gemini-2.0-flash`

6. **CORS_ORIGINS**
   - Value: `https://physical-ai-humanoid-robotics-textb-vert.vercel.app`
   - ⚠️ **No trailing slash!**

---

## Step 5: Deploy! (3-5 minutes)

1. Click **"Create Web Service"**
2. Watch the build logs (should see green checkmarks)
3. Wait for "Your service is live 🎉"
4. **Copy your URL** - looks like: `https://physical-ai-chatbot.onrender.com`

---

## Step 6: Update Vercel Frontend (2 minutes)

1. Go to **[vercel.com/dashboard](https://vercel.com/dashboard)**
2. Click on your project: `physical-ai-humanoid-robotics-textb-vert`
3. Go to **Settings** → **Environment Variables**
4. Click **"Add New"**
   - **Name:** `CHATBOT_API_URL`
   - **Value:** `https://physical-ai-chatbot.onrender.com` (your Render URL from Step 5)
   - **Environment:** Production, Preview, Development (select all)
5. Click **"Save"**
6. Go to **Deployments** tab
7. Click **"⋯"** on latest deployment → **"Redeploy"**
8. Wait for redeployment to finish (~1-2 minutes)

---

## Step 7: Test Your Chatbot! 🎉

1. Open your site: `https://physical-ai-humanoid-robotics-textb-vert.vercel.app`
2. Look for the chatbot icon in bottom right corner
3. Click it to open
4. Type: **"What is Physical AI?"**
5. Hit **Send**

### Expected Result:
- You should get an answer with source citations
- First load might take 5-10 seconds (API waking up)

### If you see an error:
- Wait 30 seconds and try again (backend might be initializing)
- Check Step 8 below for troubleshooting

---

## Step 8: Troubleshooting

### ❌ "Sorry, I encountered an error"

**Check 1: Is backend running?**
```
Visit: https://physical-ai-chatbot.onrender.com/health
Should show: {"status":"ok","version":"1.0.0"}
```

**Check 2: CORS error in browser console?**
- Press F12 → Console tab
- If you see CORS error:
  - Go to Render dashboard
  - Check `CORS_ORIGINS` matches your Vercel URL exactly
  - No trailing slash!
  - Redeploy Render service

**Check 3: Backend logs**
- Go to Render dashboard
- Click on your service
- Click "Logs" tab
- Look for error messages

**Check 4: Environment variables**
- In Render, click "Environment"
- Verify all 6 variables are set correctly
- Make sure no extra spaces or quotes

### ⏳ First request takes 30-60 seconds

This is normal for Render free tier:
- App sleeps after 15 minutes of inactivity
- First request wakes it up
- Subsequent requests are fast (1-2 seconds)

### 🔄 Backend keeps sleeping?

Consider these options:
1. Use **UptimeRobot** (free) to ping your backend every 5 minutes
   - Sign up at uptimerobot.com
   - Add monitor: https://physical-ai-chatbot.onrender.com/health
   - Interval: 5 minutes
   - Keeps your app awake!

2. Switch to **Fly.io** (no sleep, but needs credit card)
   - See main DEPLOYMENT.md guide

---

## ✅ Success Checklist

- [ ] Render service is deployed and showing "Live"
- [ ] `/health` endpoint returns `{"status":"ok"}`
- [ ] `CHATBOT_API_URL` is set in Vercel
- [ ] Frontend is redeployed
- [ ] Chatbot responds to questions
- [ ] Sources are displayed with answers

---

## 📊 What You've Deployed

```
┌─────────────────────────────────────┐
│  User visits your Docusaurus site  │
│  (Hosted on Vercel)                 │
└─────────────┬───────────────────────┘
              │
              │ Opens chatbot widget
              ▼
┌─────────────────────────────────────┐
│  widget.js asks a question          │
└─────────────┬───────────────────────┘
              │
              │ POST /api/query
              ▼
┌─────────────────────────────────────┐
│  FastAPI Backend                    │
│  (Hosted on Render - FREE)          │
│  - Receives question                │
│  - Queries Qdrant vector DB         │
│  - Generates answer with OpenAI     │
│  - Returns answer + sources         │
└─────────────┬───────────────────────┘
              │
              │ JSON response
              ▼
┌─────────────────────────────────────┐
│  Chatbot displays answer            │
│  with clickable source links        │
└─────────────────────────────────────┘
```

---

## 🎯 Next Steps

1. **Keep your API active** with UptimeRobot (prevents sleep)
2. **Monitor usage** in Render dashboard
3. **Check logs** if any issues arise
4. **Upgrade to paid tier** ($7/month) to remove sleep (optional)

---

## 💡 Pro Tips

1. **Bookmark these URLs:**
   - Render dashboard: https://dashboard.render.com
   - Vercel dashboard: https://vercel.com/dashboard
   - Your backend: https://physical-ai-chatbot.onrender.com
   - Your frontend: https://physical-ai-humanoid-robotics-textb-vert.vercel.app

2. **Check Render logs regularly** for errors or issues

3. **Your free tier includes:**
   - 750 hours/month (more than enough)
   - Unlimited bandwidth
   - Automatic HTTPS

4. **Cost to upgrade (if needed):**
   - Render: $7/month (no sleep, faster)
   - Railway: $5/month + usage
   - Fly.io: Free stays free (with limits)

---

**Need help? Check the full troubleshooting guide in `DEPLOYMENT.md`**
