# Physical AI Textbook RAG Chatbot - Backend API

Production-ready FastAPI backend for the Retrieval-Augmented Generation (RAG) chatbot that answers questions about the Physical AI & Humanoid Robotics textbook.

## Features

✅ **Zero Hallucinations**: All answers strictly grounded in textbook content
✅ **Qdrant Vector Search**: Semantic retrieval with similarity threshold
✅ **OpenAI GPT-4o-mini**: Cost-effective answer generation
✅ **Highlighted Text Mode**: Bypass retrieval for context-specific questions
✅ **Citation Generation**: Every answer links to source chapters
✅ **Chat Logging**: All interactions stored in Neon Postgres
✅ **Production Ready**: Docker, CORS, error handling, health checks

## Architecture

```
User Query → FastAPI /query endpoint
    ↓
    ├── Selection Text? → OpenAI (direct)
    │
    └── No Selection Text:
        ├── Qdrant (semantic search)
        ├── OpenAI (answer generation)
        ├── Grounding Service (citations)
        └── Neon DB (logging)
```

## Prerequisites

- Python 3.10+
- Qdrant Cloud account (Free Tier)
- OpenAI API key
- Neon Serverless Postgres account (Free Tier)
- Book content already embedded in Qdrant

## Quick Start

### 1. Clone and Install

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure Environment

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION=physical_ai_textbook

GEMINI_API_KEY=your_gemini_api_key
GEMINI_MODEL=gemini-2.0-flash

NEON_DATABASE_URL=postgresql://user:password@host:5432/database

CORS_ORIGINS=http://localhost:3000,https://yourdomain.com
```

### 3. Set Up Database

Run the SQL migration:

```bash
# Connect to your Neon database and run:
psql $NEON_DATABASE_URL < migrations/001_create_tables.sql
```

### 4. Populate Book Index

```bash
python scripts/populate_book_index.py
```

This script reads all points from Qdrant and populates the `book_index` table.

### 5. Run the Server

```bash
# Development mode with hot reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Or use the main entry point
python -m src.main
```

Server will start at: `http://localhost:8000`

- API Docs: `http://localhost:8000/docs`
- Health Check: `http://localhost:8000/health`

## API Endpoints

### POST `/api/query`

Ask a question and get a grounded answer.

**Request**:
```json
{
  "q": "What is ROS 2?",
  "top_k": 5,
  "selection_text": null
}
```

**Response**:
```json
{
  "answer": "ROS 2 is a robotics middleware...",
  "sources": [
    {
      "chapter": "Module 1: ROS 2 Fundamentals",
      "file_path": "/docs/ros2/introduction",
      "chunk_text": "ROS 2 is..."
    }
  ],
  "confidence": 0.89,
  "used_selection_text": false
}
```

### GET `/health`

Health check endpoint.

**Response**:
```json
{
  "status": "ok",
  "version": "1.0.0"
}
```

## Docker Deployment

### Build and Run

```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### Using Docker Compose

```bash
docker-compose up -d
```

## Production Deployment

### Option 1: Railway

1. Create new project on Railway.app
2. Connect GitHub repository
3. Add environment variables in Railway dashboard
4. Deploy automatically on push

### Option 2: Render

Create `render.yaml`:

```yaml
services:
  - type: web
    name: rag-chatbot-api
    env: docker
    plan: free
    envVars:
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: GEMINI_API_KEY
        sync: false
      - key: NEON_DATABASE_URL
        sync: false
```

### Option 3: Fly.io

```bash
fly launch
fly secrets set QDRANT_URL=...
fly secrets set QDRANT_API_KEY=...
fly secrets set GEMINI_API_KEY=...
fly secrets set NEON_DATABASE_URL=...
fly deploy
```

## Testing

### Manual Testing with curl

```bash
# Test health check
curl http://localhost:8000/health

# Test query (normal mode)
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"q": "What is ROS 2?", "top_k": 5}'

# Test query (selection mode)
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "q": "Explain this",
    "selection_text": "ROS 2 nodes are independent processes..."
  }'

# Test out-of-scope query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"q": "What is the capital of France?"}'
```

## Project Structure

```
backend/
├── src/
│   ├── main.py                  # FastAPI application
│   ├── config.py                # Configuration management
│   ├── database.py              # SQLAlchemy setup
│   ├── models/
│   │   ├── database.py          # ORM models (ChatLog, BookIndex)
│   │   ├── request.py           # Pydantic request models
│   │   ├── response.py          # Pydantic response models
│   │   └── retrieval.py         # RetrievedChunk dataclass
│   ├── services/
│   │   ├── qdrant_service.py    # Qdrant vector search
│   │   ├── openai_service.py    # OpenAI answer generation
│   │   ├── grounding_service.py # Citations and validation
│   │   └── database_service.py  # Chat logging
│   ├── routers/
│   │   └── query.py             # /query endpoint
│   └── utils/
│       ├── exceptions.py        # Custom exceptions
│       └── logging.py           # Structured logging
├── migrations/
│   └── 001_create_tables.sql   # Database schema
├── scripts/
│   └── populate_book_index.py  # Book index population
├── tests/                       # Test files (to be added)
├── requirements.txt             # Python dependencies
├── Dockerfile                   # Docker configuration
├── docker-compose.yml           # Docker Compose setup
├── .env.example                 # Environment variables template
└── README.md                    # This file
```

## Configuration

All configuration is managed via environment variables:

| Variable | Description | Required | Default |
|----------|-------------|----------|---------|
| `QDRANT_URL` | Qdrant cluster URL | Yes | - |
| `QDRANT_API_KEY` | Qdrant API key | Yes | - |
| `QDRANT_COLLECTION` | Collection name | Yes | - |
| `GEMINI_API_KEY` | Gemini API key | Yes | - |
| `GEMINI_MODEL` | Model to use | No | `gemini-2.0-flash` |
| `NEON_DATABASE_URL` | Postgres connection string | Yes | - |
| `CORS_ORIGINS` | Allowed origins (comma-separated) | No | `http://localhost:3000` |
| `SIMILARITY_THRESHOLD` | Minimum similarity score | No | `0.6` |
| `MAX_TOP_K` | Maximum retrieval chunks | No | `10` |

## Monitoring

The application outputs structured JSON logs to stdout:

```json
{
  "timestamp": "2025-12-09T10:30:00Z",
  "level": "INFO",
  "logger": "src.services.qdrant_service",
  "message": "Retrieved 5 chunks for query"
}
```

## Troubleshooting

### Qdrant Connection Fails

- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check collection name matches
- Test connection: `python -c "from src.services.qdrant_service import QdrantService; q = QdrantService(); print(q.test_connection())"`

### OpenAI Rate Limits

- Use `gpt-4o-mini` (higher rate limits than `gpt-4`)
- Implement caching for common queries
- Monitor usage in OpenAI dashboard

### Database Connection Issues

- Verify `NEON_DATABASE_URL` format: `postgresql://user:pass@host:5432/db`
- Check Neon dashboard for connection status
- Run migrations if tables don't exist

### Out-of-Scope Queries

This is expected behavior! The chatbot should refuse to answer questions outside the textbook scope. Adjust `SIMILARITY_THRESHOLD` if too many valid questions are rejected.

## Performance Optimization

- **Caching**: Add Redis for common queries
- **Connection Pooling**: Already configured for Neon (pool_size=5)
- **Retry Logic**: Exponential backoff for Qdrant and OpenAI
- **Concurrent Requests**: FastAPI handles async operations

## Security

- ✅ All secrets in environment variables (never in code)
- ✅ CORS configured for specific domains
- ✅ Input validation (max query length, top_k limits)
- ✅ Error messages don't expose internal details
- ✅ HTTPS enforced in production

## Contributing

1. Follow existing code structure
2. Run `black` for code formatting: `black src/`
3. Add docstrings to all functions
4. Test locally before deploying

## License

This project is part of the Physical AI & Humanoid Robotics textbook.

## Support

For issues or questions:
- Check logs: `docker-compose logs -f backend`
- Review API docs: `/docs` endpoint
- Verify environment variables are set correctly
