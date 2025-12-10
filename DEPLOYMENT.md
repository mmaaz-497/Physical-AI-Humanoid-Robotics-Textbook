# Deployment Guide

This guide explains how to deploy the Physical AI Humanoid Robotics Textbook to Vercel with the chatbot functionality.

## Architecture

The project consists of two parts:
1. **Frontend**: Docusaurus-based textbook
2. **Backend**: FastAPI-based RAG chatbot API

## Deployment Steps

### 1. Deploy the Backend (FastAPI) to Vercel

#### Option A: Deploy via Vercel Dashboard
1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Click "Add New" → "Project"
3. Import this repository
4. In "Configure Project":
   - **Root Directory**: `backend`
   - **Framework Preset**: Other
   - Click "Environment Variables" and add the following:

   ```
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-qdrant-api-key>
   QDRANT_COLLECTION=physical_ai_book
   OPENAI_API_KEY=<your-openai-api-key>
   OPENAI_MODEL=gpt-4o-mini
   CORS_ORIGINS=https://physical-ai-humanoid-robotics-textb-vert.vercel.app,https://*.vercel.app
   API_HOST=0.0.0.0
   API_PORT=8000
   SIMILARITY_THRESHOLD=0.6
   MAX_TOP_K=10
   DEFAULT_TOP_K=5
   MAX_QUERY_LENGTH=500
   MAX_SELECTION_TEXT_LENGTH=5000
   ```

5. Click "Deploy"
6. **Copy the deployment URL** (e.g., `https://your-backend.vercel.app`)

#### Option B: Deploy via Vercel CLI
```bash
cd backend
vercel --prod
```

### 2. Deploy the Frontend (Docusaurus) to Vercel

#### Update Environment Variables
In your Vercel frontend project:
1. Go to "Settings" → "Environment Variables"
2. Add the following variable:
   ```
   CHATBOT_API_URL=<your-backend-url>
   ```
   Replace `<your-backend-url>` with the URL from Step 1 (e.g., `https://your-backend.vercel.app`)

#### Deploy
The frontend will automatically redeploy when you push changes to the main branch. Or you can trigger a manual deployment from the Vercel dashboard.

## Environment Variables

### Frontend (.env or Vercel Environment Variables)
| Variable | Description | Example |
|----------|-------------|---------|
| `CHATBOT_API_URL` | Backend API URL | `https://your-backend.vercel.app` |

### Backend (.env or Vercel Environment Variables)
| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `QDRANT_URL` | ✅ | Qdrant cluster URL | `https://xxx.gcp.cloud.qdrant.io` |
| `QDRANT_API_KEY` | ✅ | Qdrant API key | `eyJhbGc...` |
| `QDRANT_COLLECTION` | ✅ | Collection name | `physical_ai_book` |
| `OPENAI_API_KEY` | ✅ | OpenAI API key | `sk-proj-...` |
| `OPENAI_MODEL` | ❌ | OpenAI model | `gpt-4o-mini` |
| `CORS_ORIGINS` | ❌ | Allowed origins (comma-separated) | `https://your-app.vercel.app` |
| `NEON_DATABASE_URL` | ❌ | PostgreSQL URL for chat logging | `postgresql://...` |

## Verification

After deployment:

1. Visit your frontend URL (e.g., `https://physical-ai-humanoid-robotics-textb-vert.vercel.app`)
2. Check browser console for:
   ```
   Chatbot widget script injected with API URL: https://your-backend.vercel.app
   ```
3. Test the chatbot by clicking the chat button
4. Verify the backend is working by visiting:
   - `https://your-backend.vercel.app/health` (should return `{"status":"ok"}`)
   - `https://your-backend.vercel.app/docs` (FastAPI documentation)

## Troubleshooting

### Book not loading
- **Error**: "Your Docusaurus site did not load properly"
- **Solution**: The baseUrl is now correctly set to `/` for Vercel. This should be resolved.

### Chatbot not working
1. **Check API URL**: Open browser console and verify the API URL is correct
2. **Check CORS**: Ensure your frontend URL is in the backend's `CORS_ORIGINS` environment variable
3. **Check Backend Health**: Visit `https://your-backend.vercel.app/health`
4. **Check Environment Variables**: Verify all required backend environment variables are set in Vercel

### GitHub Pages Deployment
For GitHub Pages, the configuration automatically detects the GitHub Pages domain and uses the correct baseUrl. No changes needed.

## Architecture Details

### How the Widget Path Works
The frontend (`src/theme/Root.js`) automatically detects the deployment environment:
- **Vercel**: Uses `/widget.js`
- **GitHub Pages**: Uses `/Physical-AI-Humanoid-Robotics-Textbook/widget.js`
- **Local**: Uses `/widget.js`

### How the API URL Works
The API URL is injected via Docusaurus `customFields`:
- In `docusaurus.config.js`, it reads from `process.env.CHATBOT_API_URL`
- In `Root.js`, it accesses via `siteConfig.customFields.chatbotApiUrl`
- For local development, it defaults to `http://localhost:8000`

## Next Steps

1. ✅ Backend deployed to Vercel
2. ✅ Frontend deployed to Vercel
3. ✅ Environment variables configured
4. ✅ CORS configured correctly
5. ✅ Test chatbot functionality

Your textbook should now be fully functional on Vercel with the chatbot working!
