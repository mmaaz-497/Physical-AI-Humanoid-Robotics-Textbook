# Implementation Plan: Integrated RAG Chatbot for Physical AI Book

**Branch**: `002-rag-chatbot-integration` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot-integration/spec.md`

## Summary

Build a Retrieval-Augmented Generation (RAG) chatbot system that enables students to ask questions about the Physical AI & Humanoid Robotics textbook and receive accurate, grounded answers exclusively from book content. The system consists of: (1) FastAPI backend with `/query` endpoint, (2) Qdrant Cloud vector database integration for semantic retrieval, (3) OpenAI GPT-4o-mini for answer generation, (4) Neon Serverless Postgres for chat logging and book index mapping, (5) Universal JavaScript widget embeddable in Docusaurus pages, (6) Highlighted text bypass mode for context-specific queries. Zero hallucinations enforced through strict grounding constraints.

## Technical Context

**Language/Version**: Python 3.10+, JavaScript ES6+
**Primary Dependencies**: FastAPI 0.104+, Qdrant Client 1.7+, OpenAI Python SDK 1.10+, Psycopg2 2.9+ (Neon), Pydantic 2.5+
**Storage**: Qdrant Cloud (vector embeddings), Neon Serverless Postgres (chat logs + book index), Static file system (widget JS)
**Testing**: Pytest 7.4+, Pytest-asyncio, Httpx (async testing), Jest (widget unit tests), Playwright (E2E)
**Target Platform**: Backend: Linux server (Docker containerized), Frontend: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
**Project Type**: Web application (backend API + frontend widget)
**Performance Goals**: <5s p95 response latency, <2s widget load time, 100 concurrent users supported
**Constraints**: Qdrant Cloud Free Tier limits, Neon Free Tier limits, OpenAI rate limits (gpt-4o-mini), 500-char query limit, 10-chunk max retrieval, zero hallucinations
**Scale/Scope**: ~1000 book chunks in Qdrant, estimated 10,000 queries/month, 36 chapters indexed, single backend service, single widget component

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance

**I. Accuracy (NON-NEGOTIABLE)** ✅ PASS
- All answers grounded exclusively on retrieved book context (enforced via system prompt)
- Retrieval limited to book content only (Qdrant collection scoped to Physical AI textbook)
- Citation requirement: every answer must reference source chapters
- Similarity threshold (0.6) prevents low-confidence responses

**II. Clarity and Accessibility** ✅ PASS
- Widget accessible via keyboard navigation (Tab, Enter, Esc)
- Clear error messages for all failure modes (API down, rate limit, invalid query)
- User-friendly refusal messages when questions are out of scope

**III. Hierarchical Organization** ✅ PASS
- Backend structured as modular FastAPI application (routers, services, models)
- Widget components follow separation of concerns (UI, API service, state management)
- Database schema normalized (chat_logs, book_index separate tables)

**IV. Reproducibility** ✅ PASS
- All environment variables documented in README
- Docker Compose for local development setup
- Database migrations tracked with Alembic
- Widget build process documented

**V. Real-World Rigor** ✅ PASS
- Production-ready error handling (timeouts, retries, graceful degradation)
- Observability: structured logging to stdout/stderr
- CORS configuration for cross-origin requests
- HTTPS enforced in production

**VI. Consistency Across System** ✅ PASS
- API follows RESTful conventions
- Pydantic models for request/response validation
- Consistent code formatting (Black for Python, Prettier for JavaScript)

**VII. Automation-First Development** ✅ PASS
- Following /sp.plan → /sp.tasks → /sp.implement workflow
- Automated testing (unit, integration, E2E)
- CI/CD ready (deployment scripts included)

### Constitution Compliance Summary

**Status**: ✅ ALL GATES PASSED

No constitution violations detected. The RAG chatbot system aligns with all seven core principles and technical standards.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot-integration/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (Qdrant, OpenAI, Neon research)
├── data-model.md        # Database schemas (chat_logs, book_index)
├── api-contract.md      # FastAPI endpoint specifications
├── quickstart.md        # Local development setup guide
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Web Application Structure (Backend + Frontend)

backend/
├── src/
│   ├── main.py                      # FastAPI application entry point
│   ├── config.py                    # Environment variable loading
│   ├── models/
│   │   ├── request.py               # Pydantic request models (QueryRequest)
│   │   ├── response.py              # Pydantic response models (QueryResponse)
│   │   └── database.py              # SQLAlchemy ORM models (ChatLog, BookIndex)
│   ├── services/
│   │   ├── qdrant_service.py        # Qdrant client, retrieval logic
│   │   ├── openai_service.py        # OpenAI client, answer generation
│   │   ├── database_service.py      # Neon Postgres operations
│   │   └── grounding_service.py     # Answer validation, citation generation
│   ├── routers/
│   │   └── query.py                 # /query endpoint implementation
│   └── utils/
│       ├── logging.py               # Structured logging setup
│       └── exceptions.py            # Custom exception classes
├── tests/
│   ├── unit/
│   │   ├── test_qdrant_service.py
│   │   ├── test_openai_service.py
│   │   ├── test_grounding_service.py
│   │   └── test_database_service.py
│   ├── integration/
│   │   ├── test_query_endpoint.py
│   │   └── test_full_rag_flow.py
│   └── e2e/
│       └── test_widget_integration.py
├── migrations/                      # Alembic database migrations
│   └── versions/
│       ├── 001_create_chat_logs.py
│       └── 002_create_book_index.py
├── Dockerfile
├── docker-compose.yml               # Local development with Postgres
├── requirements.txt
├── pyproject.toml                   # Project metadata, dependencies
└── README.md

frontend/
├── src/
│   ├── widget.js                    # Main widget component
│   ├── api-service.js               # HTTP client for /query endpoint
│   ├── text-selection.js            # Highlighted text capture logic
│   ├── ui-components.js             # Chat UI elements (message list, input, button)
│   └── styles.css                   # Widget styling (scoped, no conflicts)
├── tests/
│   ├── widget.test.js               # Jest unit tests
│   └── e2e/
│       └── chatbot-flow.spec.js     # Playwright E2E tests
├── build/
│   └── widget.min.js                # Production bundle (generated)
├── package.json
├── webpack.config.js                # Build configuration
└── README.md

docs/                                # Docusaurus integration
└── src/
    └── theme/
        └── Root.js                  # Inject widget on all pages
```

**Structure Decision**: Web application structure selected because the feature requires both a backend API (FastAPI) and a frontend widget (JavaScript). Backend handles RAG logic, database operations, and external API calls. Frontend provides user interface embedded in Docusaurus site.

## Complexity Tracking

No constitution violations requiring justification.

---

# 1. WORKSTREAM BREAKDOWN

## Workstream 1: Backend FastAPI Service

**Owner**: Backend workstream
**Duration**: Estimated 3-5 implementation cycles
**Description**: Build FastAPI application with `/query` endpoint, environment variable management, logging, error handling, and CORS configuration.

**Components**:
- FastAPI application setup (main.py, config.py)
- Request/response Pydantic models
- Query router with input validation
- Structured logging
- Custom exception handling
- CORS middleware
- Health check endpoint

**Deliverables**:
- `backend/src/main.py` - FastAPI app entry point
- `backend/src/config.py` - Environment variable loader
- `backend/src/models/request.py` - QueryRequest model
- `backend/src/models/response.py` - QueryResponse model
- `backend/src/routers/query.py` - /query endpoint
- `backend/src/utils/logging.py` - Logging configuration
- `backend/src/utils/exceptions.py` - Custom exceptions

---

## Workstream 2: Qdrant Integration Layer

**Owner**: Backend workstream
**Duration**: Estimated 2-3 implementation cycles
**Description**: Implement Qdrant Cloud client, semantic similarity search, embedding generation for user queries, and fallback handling when no relevant chunks are found.

**Components**:
- Qdrant client initialization
- Query embedding generation (text-embedding-3-small)
- Semantic search with configurable top_k
- Similarity score threshold filtering (0.6)
- Metadata extraction from retrieved chunks
- Error handling (timeouts, connection failures)

**Deliverables**:
- `backend/src/services/qdrant_service.py` - Qdrant operations
- Unit tests with mocked Qdrant responses
- Configuration for QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION

---

## Workstream 3: OpenAI Answer Generation Layer

**Owner**: Backend workstream
**Duration**: Estimated 2-3 implementation cycles
**Description**: Implement OpenAI client for GPT-4o-mini, prompt engineering for grounding, answer generation from retrieved context, and response validation.

**Components**:
- OpenAI client initialization
- System prompt design (enforce grounding, citations)
- Answer generation from retrieved chunks or selection_text
- Token usage tracking
- Rate limit handling
- Retry logic with exponential backoff

**Deliverables**:
- `backend/src/services/openai_service.py` - OpenAI operations
- Grounding prompt template
- Unit tests with mocked OpenAI responses
- Configuration for OPENAI_API_KEY, OPENAI_MODEL

---

## Workstream 4: Answer Grounding & Citation Service

**Owner**: Backend workstream
**Duration**: Estimated 2-3 implementation cycles
**Description**: Validate that generated answers are grounded in retrieved context, generate citations with chapter links, and handle out-of-scope queries.

**Components**:
- Citation extraction from retrieved chunk metadata
- Chapter URL generation from book_index lookups
- Out-of-scope detection (low similarity scores)
- Refusal message generation
- Confidence score calculation

**Deliverables**:
- `backend/src/services/grounding_service.py` - Grounding logic
- Citation formatting functions
- Unit tests for grounding validation

---

## Workstream 5: Neon Postgres Integration

**Owner**: Backend workstream
**Duration**: Estimated 3-4 implementation cycles
**Description**: Set up Neon Serverless Postgres connection, create database schemas, implement Alembic migrations, and build services for logging and book index queries.

**Components**:
- SQLAlchemy ORM models (ChatLog, BookIndex)
- Alembic migration setup
- Database connection pooling
- Chat log insertion service
- Book index query service (qdrant_id → chapter URL)
- Migration files (create_chat_logs, create_book_index)

**Deliverables**:
- `backend/src/models/database.py` - SQLAlchemy models
- `backend/src/services/database_service.py` - Database operations
- `backend/migrations/versions/001_create_chat_logs.py`
- `backend/migrations/versions/002_create_book_index.py`
- Configuration for NEON_DATABASE_URL

---

## Workstream 6: Frontend Widget Development

**Owner**: Frontend workstream
**Duration**: Estimated 4-5 implementation cycles
**Description**: Build universal JavaScript widget with chat UI, API service wrapper, highlighted text selection, conversation history, and error handling.

**Components**:
- Widget main component (open/close, state management)
- Chat UI (message list, input field, send button)
- API service wrapper (fetch calls to /query endpoint)
- Text selection capture logic
- Conversation history display
- Loading and error states
- Responsive styling (desktop, tablet, mobile)

**Deliverables**:
- `frontend/src/widget.js` - Main widget component
- `frontend/src/api-service.js` - HTTP client
- `frontend/src/text-selection.js` - Selection capture
- `frontend/src/ui-components.js` - UI elements
- `frontend/src/styles.css` - Widget styles
- Webpack build configuration

---

## Workstream 7: Docusaurus Integration

**Owner**: Frontend workstream
**Duration**: Estimated 2 implementation cycles
**Description**: Integrate chatbot widget into Docusaurus site by injecting it via theme customization (Root.js swizzling) or static script injection.

**Components**:
- Docusaurus theme swizzling (Root.js)
- Widget script injection
- Environment variable configuration for backend API URL
- Testing on all chapter pages

**Deliverables**:
- `docs/src/theme/Root.js` - Widget injection point
- Configuration for CHATBOT_API_URL
- Integration testing on Docusaurus site

---

## Workstream 8: Local Development Environment

**Owner**: DevOps workstream
**Duration**: Estimated 2 implementation cycles
**Description**: Create Docker Compose setup for local development with FastAPI backend, local Postgres (optional), and environment variable management.

**Components**:
- Dockerfile for FastAPI backend
- Docker Compose configuration
- .env.example file with all required variables
- README with setup instructions

**Deliverables**:
- `backend/Dockerfile`
- `backend/docker-compose.yml`
- `.env.example`
- `backend/README.md` - Setup guide

---

## Workstream 9: Production Deployment

**Owner**: DevOps workstream
**Duration**: Estimated 2-3 implementation cycles
**Description**: Deploy FastAPI backend to hosting platform (Railway/Render/Fly.io), configure environment variables, set up HTTPS, and enable CORS.

**Components**:
- Platform-specific deployment configuration
- Environment variable setup in hosting platform
- HTTPS certificate configuration
- CORS whitelist configuration (Docusaurus domain)
- Health check endpoint configuration

**Deliverables**:
- Deployment configuration files (railway.json, render.yaml, or fly.toml)
- Deployment instructions in README
- Production environment variable checklist

---

## Workstream 10: Testing & Validation

**Owner**: QA workstream
**Duration**: Ongoing throughout implementation
**Description**: Implement unit tests, integration tests, E2E tests, and manual validation for all components.

**Components**:
- Unit tests (Qdrant, OpenAI, database, grounding services)
- Integration tests (/query endpoint, full RAG flow)
- E2E tests (widget interaction with backend)
- Manual testing checklist (grounding validation, out-of-scope queries)

**Deliverables**:
- `backend/tests/unit/*` - Unit test suite
- `backend/tests/integration/*` - Integration test suite
- `frontend/tests/e2e/*` - E2E test suite
- Manual testing checklist document

---

# 2. MILESTONES

## Milestone 1: Environment & Project Scaffolding

**Description**: Set up backend and frontend project structure, initialize dependencies, configure environment variables, and validate connections to external services (Qdrant, OpenAI, Neon).

**Deliverables**:
- Backend FastAPI project initialized with pyproject.toml, requirements.txt
- Frontend widget project initialized with package.json, webpack.config.js
- .env.example file with all required environment variables
- Successful connection tests to Qdrant Cloud, OpenAI API, Neon Postgres
- Git repository structure created

**Dependencies**:
- None (starting point)

**Acceptance Criteria**:
- [ ] Backend runs `uvicorn src.main:app --reload` without errors
- [ ] Frontend builds with `npm run build` without errors
- [ ] Qdrant client connects to collection successfully
- [ ] OpenAI client validates API key successfully
- [ ] Neon database connection established

---

## Milestone 2: Database Schema & Migrations

**Description**: Create Neon Postgres schemas for `chat_logs` and `book_index` tables, implement Alembic migrations, and populate book_index with Qdrant ID → chapter URL mappings.

**Deliverables**:
- SQLAlchemy models for ChatLog and BookIndex
- Alembic migration files (001_create_chat_logs, 002_create_book_index)
- book_index table populated with mappings
- Database service functions for insert and query operations

**Dependencies**:
- Milestone 1 (database connection established)

**Acceptance Criteria**:
- [ ] Migrations run successfully with `alembic upgrade head`
- [ ] chat_logs table created with schema: id, question, answer, timestamp, retrieval_metadata, model_used
- [ ] book_index table created with schema: qdrant_id, chapter_title, file_path, section_heading
- [ ] Sample book_index entries inserted and queryable
- [ ] Database service can insert chat logs and query book index

---

## Milestone 3: RAG Core - Retrieval Layer

**Description**: Implement Qdrant retrieval service with query embedding, semantic search, similarity threshold filtering, and metadata extraction.

**Deliverables**:
- Qdrant service with search function
- Query embedding generation using text-embedding-3-small
- Similarity score threshold (0.6) implementation
- Retrieved chunk metadata extraction
- Unit tests with mocked Qdrant responses

**Dependencies**:
- Milestone 1 (Qdrant connection established)
- Milestone 2 (book_index available for citation lookups)

**Acceptance Criteria**:
- [ ] Query "What is ROS 2?" retrieves relevant chunks from Qdrant
- [ ] Similarity scores below 0.6 are filtered out
- [ ] Retrieved chunks include metadata: qdrant_id, chunk_text, similarity_score
- [ ] Unit tests pass with 100% coverage for qdrant_service.py
- [ ] Error handling works for Qdrant timeouts and connection failures

---

## Milestone 4: RAG Core - Generation Layer

**Description**: Implement OpenAI service for answer generation with strict grounding prompt, citation generation, and out-of-scope handling.

**Deliverables**:
- OpenAI service with answer generation function
- System prompt enforcing grounding and citations
- Answer validation and citation extraction
- Grounding service for out-of-scope detection
- Unit tests with mocked OpenAI responses

**Dependencies**:
- Milestone 3 (retrieval layer provides context chunks)

**Acceptance Criteria**:
- [ ] System prompt instructs: "Answer ONLY using the provided context. Cite sources."
- [ ] Generated answers include citations to source chapters
- [ ] Out-of-scope queries (low similarity) trigger refusal message
- [ ] Unit tests pass with 100% coverage for openai_service.py and grounding_service.py
- [ ] No hallucinations detected in 20 test queries

---

## Milestone 5: Backend API - /query Endpoint

**Description**: Implement FastAPI /query endpoint with request validation, RAG orchestration (retrieval + generation), selection_text bypass mode, and chat logging.

**Deliverables**:
- /query POST endpoint accepting QueryRequest (q, top_k, selection_text)
- Pydantic models for request and response
- RAG orchestration logic
- Selection_text bypass mode (skip Qdrant if provided)
- Chat log insertion after response generation
- CORS middleware configuration

**Dependencies**:
- Milestone 2 (database logging ready)
- Milestone 3 (retrieval layer ready)
- Milestone 4 (generation layer ready)

**Acceptance Criteria**:
- [ ] POST /query with {"q": "What is ROS 2?"} returns valid response
- [ ] Response includes: answer, sources (array), confidence
- [ ] POST /query with {"q": "test", "selection_text": "ROS 2 is..."} bypasses Qdrant
- [ ] All queries are logged to chat_logs table
- [ ] CORS allows requests from Docusaurus domain
- [ ] Integration tests pass for /query endpoint

---

## Milestone 6: Frontend Widget - Core UI

**Description**: Build chatbot widget UI with open/close functionality, message history display, input field, send button, and loading states.

**Deliverables**:
- widget.js main component
- ui-components.js (message list, input, button)
- styles.css (scoped styling)
- Open/close widget functionality
- Conversation history rendering

**Dependencies**:
- Milestone 5 (backend /query endpoint available)

**Acceptance Criteria**:
- [ ] Widget icon appears in bottom-right corner
- [ ] Clicking icon opens/closes widget
- [ ] Input field accepts text up to 500 characters
- [ ] Send button triggers API call
- [ ] Loading spinner displays during API call
- [ ] Conversation history displays user questions and chatbot answers

---

## Milestone 7: Frontend Widget - API Integration & Selection Mode

**Description**: Integrate widget with backend /query endpoint, implement highlighted text selection capture, and error handling.

**Deliverables**:
- api-service.js (HTTP client for /query)
- text-selection.js (capture highlighted text)
- Error message display for failure modes
- Selection_text parameter integration

**Dependencies**:
- Milestone 5 (backend API ready)
- Milestone 6 (widget UI ready)

**Acceptance Criteria**:
- [ ] Widget sends POST request to /query endpoint
- [ ] Response displays in conversation history within 5 seconds
- [ ] User can highlight text on page and ask question
- [ ] Highlighted text passed as selection_text parameter
- [ ] Error messages display for: API down, rate limit, network error
- [ ] Widget works on desktop, tablet, and mobile viewports

---

## Milestone 8: Docusaurus Integration

**Description**: Embed chatbot widget into Docusaurus site by injecting script via theme customization or static file.

**Deliverables**:
- docs/src/theme/Root.js (swizzled Docusaurus theme component)
- Widget script injection code
- Configuration for CHATBOT_API_URL environment variable
- Testing on all chapter pages

**Dependencies**:
- Milestone 7 (widget fully functional)
- Feature 001 (Docusaurus site deployed)

**Acceptance Criteria**:
- [ ] Widget appears on all Docusaurus chapter pages
- [ ] Widget loads within 2 seconds of page load
- [ ] Widget does not conflict with Docusaurus JavaScript or CSS
- [ ] Widget uses production backend API URL
- [ ] Manual testing confirms widget works on 5 random chapter pages

---

## Milestone 9: Deployment & Production Readiness

**Description**: Deploy FastAPI backend to production hosting platform, configure environment variables, enable HTTPS and CORS, and validate production setup.

**Deliverables**:
- Backend deployed to Railway/Render/Fly.io
- Environment variables configured in hosting platform
- HTTPS enabled
- CORS whitelist configured
- Health check endpoint tested

**Dependencies**:
- Milestone 5 (backend API complete)
- Milestone 8 (widget integrated in Docusaurus)

**Acceptance Criteria**:
- [ ] Backend accessible via HTTPS URL (e.g., https://chatbot-api.railway.app)
- [ ] /query endpoint returns valid responses in production
- [ ] CORS allows requests from Docusaurus production domain
- [ ] Environment variables loaded correctly (Qdrant, OpenAI, Neon credentials)
- [ ] Health check endpoint (/health) returns 200 OK

---

## Milestone 10: Testing & Validation

**Description**: Complete unit test coverage, integration tests, E2E tests, and manual validation checklist for grounding accuracy and zero hallucinations.

**Deliverables**:
- Unit test suite (>90% coverage)
- Integration test suite
- E2E test suite (Playwright)
- Manual testing checklist completed
- Grounding validation report (100 test queries)

**Dependencies**:
- Milestone 9 (production deployment complete)

**Acceptance Criteria**:
- [ ] All unit tests pass (backend and frontend)
- [ ] Integration tests pass for /query endpoint and full RAG flow
- [ ] E2E tests pass for widget interaction with backend
- [ ] Manual testing checklist: 100 in-scope questions answered correctly (95%+ accuracy)
- [ ] Manual testing checklist: 20 out-of-scope questions refused correctly (100%)
- [ ] Zero hallucinations detected in validation queries

---

# 3. DETAILED TASK BREAKDOWN

## 3.1 Backend FastAPI Service

### Task 3.1.1: Initialize FastAPI Project

**Subtasks**:

1. **Create backend directory structure**
   - Files to create:
     - `backend/src/main.py`
     - `backend/src/config.py`
     - `backend/pyproject.toml`
     - `backend/requirements.txt`
     - `backend/.env.example`
     - `backend/README.md`
   - Commands to run:
     ```bash
     mkdir -p backend/src backend/tests
     cd backend
     python -m venv venv
     source venv/bin/activate  # On Windows: venv\Scripts\activate
     ```
   - Acceptance criteria:
     - Directory structure exists
     - Virtual environment created

2. **Install FastAPI and dependencies**
   - Files to modify:
     - `backend/requirements.txt` - Add: fastapi, uvicorn[standard], pydantic, pydantic-settings, python-dotenv
   - Commands to run:
     ```bash
     pip install -r requirements.txt
     ```
   - Acceptance criteria:
     - Dependencies installed successfully
     - `pip list` shows FastAPI, Uvicorn, Pydantic

3. **Create FastAPI application entry point**
   - Files to create:
     - `backend/src/main.py` - FastAPI app initialization, CORS middleware
   - Content:
     - Import FastAPI, CORSMiddleware
     - Create app instance
     - Configure CORS (allow Docusaurus domain)
     - Add health check endpoint: GET /health → {"status": "ok"}
   - Commands to run:
     ```bash
     uvicorn src.main:app --reload
     ```
   - Acceptance criteria:
     - Server starts on http://localhost:8000
     - GET /health returns 200 OK
     - OpenAPI docs accessible at /docs

4. **Create environment variable configuration**
   - Files to create:
     - `backend/src/config.py` - Load environment variables using Pydantic BaseSettings
     - `backend/.env.example` - Template with all required variables
   - Content (config.py):
     - Define Settings class with fields: QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION, OPENAI_API_KEY, OPENAI_MODEL, NEON_DATABASE_URL
     - Use Pydantic BaseSettings to load from .env file
   - Content (.env.example):
     ```
     QDRANT_URL=https://your-cluster.qdrant.io
     QDRANT_API_KEY=your_api_key
     QDRANT_COLLECTION=physical_ai_textbook
     OPENAI_API_KEY=sk-your_key
     OPENAI_MODEL=gpt-4o-mini
     NEON_DATABASE_URL=postgresql://user:pass@host/db
     ```
   - Acceptance criteria:
     - Settings class loads variables from .env file
     - Missing required variables raise validation errors

---

### Task 3.1.2: Create Pydantic Request/Response Models

**Subtasks**:

1. **Create QueryRequest model**
   - Files to create:
     - `backend/src/models/request.py`
   - Content:
     - QueryRequest class with fields:
       - `q`: str (required, max_length=500)
       - `top_k`: int (optional, default=5, ge=1, le=10)
       - `selection_text`: Optional[str] (optional, max_length=5000)
   - Acceptance criteria:
     - Model validates query length (max 500 chars)
     - top_k capped at 10
     - selection_text is optional

2. **Create QueryResponse model**
   - Files to create:
     - `backend/src/models/response.py`
   - Content:
     - CitationSource class with fields: chapter, file_path, chunk_text
     - QueryResponse class with fields:
       - `answer`: str
       - `sources`: List[CitationSource]
       - `confidence`: float (0-1)
   - Acceptance criteria:
     - Model serializes to JSON correctly
     - Nested CitationSource objects handled

---

### Task 3.1.3: Implement /query Endpoint Router

**Subtasks**:

1. **Create query router**
   - Files to create:
     - `backend/src/routers/query.py`
   - Content:
     - POST /query endpoint
     - Accept QueryRequest, return QueryResponse
     - Validate request body
     - Call RAG orchestration service (placeholder for now)
     - Return mock response
   - Acceptance criteria:
     - POST /query accepts JSON body
     - Returns 200 OK with QueryResponse
     - Returns 422 Unprocessable Entity for invalid input

2. **Register router in main.py**
   - Files to modify:
     - `backend/src/main.py`
   - Content:
     - Import query router
     - Register with app.include_router(query_router)
   - Commands to run:
     ```bash
     curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d '{"q": "test"}'
     ```
   - Acceptance criteria:
     - /query endpoint accessible
     - Returns valid QueryResponse

---

### Task 3.1.4: Implement Logging and Exception Handling

**Subtasks**:

1. **Configure structured logging**
   - Files to create:
     - `backend/src/utils/logging.py`
   - Content:
     - Configure Python logging to stdout
     - JSON-formatted logs (timestamp, level, message, request_id)
     - Log all incoming requests and responses
   - Acceptance criteria:
     - Logs appear in stdout during development
     - Log format is structured JSON

2. **Create custom exception classes**
   - Files to create:
     - `backend/src/utils/exceptions.py`
   - Content:
     - QdrantConnectionError
     - OpenAIRateLimitError
     - OutOfScopeQueryError
     - DatabaseError
   - Acceptance criteria:
     - Exceptions inherit from base Exception
     - Include error message and optional context

3. **Add global exception handler**
   - Files to modify:
     - `backend/src/main.py`
   - Content:
     - @app.exception_handler for each custom exception
     - Return user-friendly error messages with appropriate status codes
   - Acceptance criteria:
     - Exceptions return JSON error responses
     - Status codes: 503 (service unavailable), 429 (rate limit), 400 (bad request)

---

## 3.2 Qdrant Integration Layer

### Task 3.2.1: Set Up Qdrant Client

**Subtasks**:

1. **Install Qdrant client library**
   - Files to modify:
     - `backend/requirements.txt` - Add: qdrant-client>=1.7.0
   - Commands to run:
     ```bash
     pip install qdrant-client
     ```
   - Acceptance criteria:
     - qdrant-client library installed

2. **Create Qdrant service class**
   - Files to create:
     - `backend/src/services/qdrant_service.py`
   - Content:
     - QdrantService class
     - __init__: Initialize Qdrant client with URL, API key from config
     - test_connection(): Verify connection to collection
   - Commands to run:
     ```bash
     python -c "from src.services.qdrant_service import QdrantService; qs = QdrantService(); qs.test_connection()"
     ```
   - Acceptance criteria:
     - Qdrant client connects successfully
     - Collection exists and is accessible

---

### Task 3.2.2: Implement Query Embedding Generation

**Subtasks**:

1. **Install OpenAI library for embeddings**
   - Files to modify:
     - `backend/requirements.txt` - Add: openai>=1.10.0
   - Commands to run:
     ```bash
     pip install openai
     ```
   - Acceptance criteria:
     - openai library installed

2. **Create embedding generation function**
   - Files to modify:
     - `backend/src/services/qdrant_service.py`
   - Content:
     - generate_query_embedding(query: str) -> List[float]
     - Use OpenAI text-embedding-3-small model
     - Return 1536-dimensional vector
   - Acceptance criteria:
     - Function returns 1536-dimensional embedding
     - Embeddings are consistent for same input

---

### Task 3.2.3: Implement Semantic Search with Similarity Threshold

**Subtasks**:

1. **Create semantic search function**
   - Files to modify:
     - `backend/src/services/qdrant_service.py`
   - Content:
     - search(query: str, top_k: int = 5, similarity_threshold: float = 0.6) -> List[RetrievedChunk]
     - Generate query embedding
     - Search Qdrant collection
     - Filter results by similarity score >= threshold
     - Extract metadata: qdrant_id, chunk_text, similarity_score
     - Return list of RetrievedChunk objects
   - Acceptance criteria:
     - Search returns top_k chunks sorted by similarity
     - Chunks below threshold are filtered out
     - Metadata extracted correctly

2. **Define RetrievedChunk data class**
   - Files to create:
     - `backend/src/models/retrieval.py`
   - Content:
     - RetrievedChunk dataclass with fields: qdrant_id, chunk_text, similarity_score, metadata
   - Acceptance criteria:
     - Data class defined and usable

---

### Task 3.2.4: Implement Error Handling for Qdrant Operations

**Subtasks**:

1. **Add timeout and retry logic**
   - Files to modify:
     - `backend/src/services/qdrant_service.py`
   - Content:
     - Wrap Qdrant API calls in try/except
     - Catch connection timeouts, API errors
     - Raise QdrantConnectionError with user-friendly message
     - Add retry logic with exponential backoff (max 3 retries)
   - Acceptance criteria:
     - Timeouts handled gracefully
     - Retries attempted on transient failures
     - Permanent failures raise clear exceptions

---

## 3.3 OpenAI Answer Generation Layer

### Task 3.3.1: Create OpenAI Service

**Subtasks**:

1. **Create OpenAI service class**
   - Files to create:
     - `backend/src/services/openai_service.py`
   - Content:
     - OpenAIService class
     - __init__: Initialize OpenAI client with API key from config
     - test_connection(): Validate API key
   - Acceptance criteria:
     - OpenAI client initialized successfully
     - API key validated

---

### Task 3.3.2: Design Grounding System Prompt

**Subtasks**:

1. **Create grounding prompt template**
   - Files to modify:
     - `backend/src/services/openai_service.py`
   - Content:
     - GROUNDING_SYSTEM_PROMPT constant:
       ```
       You are a helpful assistant answering questions about the Physical AI & Humanoid Robotics textbook.

       CRITICAL RULES:
       1. Answer ONLY using the provided context below. Do NOT use external knowledge.
       2. If the context does not contain the answer, respond with: "I couldn't find information on this topic in the textbook."
       3. Cite your sources by referencing the chapter name in your answer.
       4. Be concise and accurate.

       Context:
       {context}

       Question: {question}
       ```
   - Acceptance criteria:
     - Prompt enforces grounding constraints
     - Placeholders for context and question

---

### Task 3.3.3: Implement Answer Generation Function

**Subtasks**:

1. **Create generate_answer function**
   - Files to modify:
     - `backend/src/services/openai_service.py`
   - Content:
     - generate_answer(question: str, context_chunks: List[RetrievedChunk]) -> str
     - Format context chunks into string
     - Build prompt with GROUNDING_SYSTEM_PROMPT
     - Call OpenAI ChatCompletion API (gpt-4o-mini)
     - Extract answer from response
     - Return answer text
   - Acceptance criteria:
     - Function returns generated answer
     - Answer respects grounding constraints

2. **Handle selection_text bypass mode**
   - Files to modify:
     - `backend/src/services/openai_service.py`
   - Content:
     - generate_answer_from_selection(question: str, selection_text: str) -> str
     - Use selection_text as context instead of retrieved chunks
     - Same grounding prompt structure
   - Acceptance criteria:
     - Selection text used as sole context
     - Answer generated without Qdrant retrieval

---

### Task 3.3.4: Implement Rate Limit and Error Handling

**Subtasks**:

1. **Add rate limit handling**
   - Files to modify:
     - `backend/src/services/openai_service.py`
   - Content:
     - Catch OpenAI RateLimitError
     - Raise OpenAIRateLimitError with retry-after header
     - Add exponential backoff retry logic
   - Acceptance criteria:
     - Rate limits handled gracefully
     - User receives clear error message

2. **Add token usage tracking**
   - Files to modify:
     - `backend/src/services/openai_service.py`
   - Content:
     - Log token usage from OpenAI response
     - Track prompt tokens and completion tokens
   - Acceptance criteria:
     - Token usage logged for monitoring

---

## 3.4 Answer Grounding & Citation Service

### Task 3.4.1: Create Grounding Service

**Subtasks**:

1. **Create grounding service class**
   - Files to create:
     - `backend/src/services/grounding_service.py`
   - Content:
     - GroundingService class
     - validate_answer(answer: str, context_chunks: List[RetrievedChunk]) -> bool
     - Heuristic: Check if answer mentions content from context chunks
   - Acceptance criteria:
     - Basic validation implemented

---

### Task 3.4.2: Implement Citation Generation

**Subtasks**:

1. **Create citation extraction function**
   - Files to modify:
     - `backend/src/services/grounding_service.py`
   - Content:
     - generate_citations(context_chunks: List[RetrievedChunk], book_index_service) -> List[CitationSource]
     - For each chunk, lookup chapter info from book_index using qdrant_id
     - Build CitationSource objects with chapter, file_path, chunk_text
     - Return list of citations
   - Acceptance criteria:
     - Citations include chapter name and file path
     - Qdrant ID mapped to chapter URL via book_index

---

### Task 3.4.3: Implement Out-of-Scope Detection

**Subtasks**:

1. **Create out-of-scope detection function**
   - Files to modify:
     - `backend/src/services/grounding_service.py`
   - Content:
     - is_out_of_scope(context_chunks: List[RetrievedChunk], similarity_threshold: float = 0.6) -> bool
     - Return True if all chunks below threshold or no chunks retrieved
   - Acceptance criteria:
     - Returns True for low-similarity queries
     - Returns False for high-similarity queries

2. **Create refusal message generator**
   - Files to modify:
     - `backend/src/services/grounding_service.py`
   - Content:
     - generate_refusal_message() -> str
     - Return: "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content. Your question appears to be outside the book's scope."
   - Acceptance criteria:
     - Refusal message is user-friendly

---

## 3.5 Neon Postgres Integration

### Task 3.5.1: Set Up SQLAlchemy Models

**Subtasks**:

1. **Install SQLAlchemy and Psycopg2**
   - Files to modify:
     - `backend/requirements.txt` - Add: sqlalchemy>=2.0, psycopg2-binary>=2.9, alembic>=1.12
   - Commands to run:
     ```bash
     pip install sqlalchemy psycopg2-binary alembic
     ```
   - Acceptance criteria:
     - Libraries installed successfully

2. **Create database connection setup**
   - Files to create:
     - `backend/src/database.py`
   - Content:
     - Create SQLAlchemy engine using NEON_DATABASE_URL
     - Create session factory
     - Define Base for declarative models
   - Acceptance criteria:
     - Database connection established

---

### Task 3.5.2: Create SQLAlchemy ORM Models

**Subtasks**:

1. **Create ChatLog model**
   - Files to create:
     - `backend/src/models/database.py`
   - Content:
     - ChatLog class inheriting from Base
     - Fields:
       - id: Integer, primary key, autoincrement
       - question: Text, not null
       - answer: Text, not null
       - timestamp: DateTime, default=now
       - retrieval_metadata: JSON (Qdrant chunk IDs, similarity scores)
       - model_used: String (e.g., "gpt-4o-mini")
   - Acceptance criteria:
     - Model defined correctly

2. **Create BookIndex model**
   - Files to modify:
     - `backend/src/models/database.py`
   - Content:
     - BookIndex class inheriting from Base
     - Fields:
       - id: Integer, primary key
       - qdrant_id: String, unique, not null
       - chapter_title: String, not null
       - file_path: String, not null (e.g., /docs/ros2/nodes-and-topics)
       - section_heading: String, optional
   - Acceptance criteria:
     - Model defined correctly

---

### Task 3.5.3: Set Up Alembic Migrations

**Subtasks**:

1. **Initialize Alembic**
   - Commands to run:
     ```bash
     cd backend
     alembic init migrations
     ```
   - Files created:
     - `backend/alembic.ini`
     - `backend/migrations/env.py`
   - Acceptance criteria:
     - Alembic directory structure created

2. **Configure Alembic to use Neon URL**
   - Files to modify:
     - `backend/alembic.ini` - Set sqlalchemy.url to use NEON_DATABASE_URL
     - `backend/migrations/env.py` - Import models, configure target_metadata
   - Acceptance criteria:
     - Alembic configured correctly

---

### Task 3.5.4: Create Database Migrations

**Subtasks**:

1. **Create chat_logs table migration**
   - Commands to run:
     ```bash
     alembic revision --autogenerate -m "create_chat_logs"
     ```
   - Files created:
     - `backend/migrations/versions/001_create_chat_logs.py`
   - Content:
     - Upgrade: CREATE TABLE chat_logs with columns
     - Downgrade: DROP TABLE chat_logs
   - Acceptance criteria:
     - Migration file generated

2. **Create book_index table migration**
   - Commands to run:
     ```bash
     alembic revision --autogenerate -m "create_book_index"
     ```
   - Files created:
     - `backend/migrations/versions/002_create_book_index.py`
   - Content:
     - Upgrade: CREATE TABLE book_index with columns
     - Downgrade: DROP TABLE book_index
   - Acceptance criteria:
     - Migration file generated

3. **Run migrations**
   - Commands to run:
     ```bash
     alembic upgrade head
     ```
   - Acceptance criteria:
     - Tables created in Neon database
     - Verify with: SELECT * FROM chat_logs; SELECT * FROM book_index;

---

### Task 3.5.5: Implement Database Service Functions

**Subtasks**:

1. **Create database service class**
   - Files to create:
     - `backend/src/services/database_service.py`
   - Content:
     - DatabaseService class
     - insert_chat_log(question, answer, retrieval_metadata, model_used)
     - query_book_index(qdrant_id) -> BookIndex
   - Acceptance criteria:
     - Functions work as expected

2. **Test database operations**
   - Commands to run:
     ```bash
     python -c "from src.services.database_service import DatabaseService; db = DatabaseService(); db.insert_chat_log('test', 'test answer', {}, 'gpt-4o-mini')"
     ```
   - Acceptance criteria:
     - Chat log inserted successfully
     - Query book_index returns correct chapter info

---

### Task 3.5.6: Populate book_index Table

**Subtasks**:

1. **Create book index population script**
   - Files to create:
     - `backend/scripts/populate_book_index.py`
   - Content:
     - Read Qdrant collection metadata
     - Map each Qdrant ID to chapter title and file path
     - Insert into book_index table
   - Commands to run:
     ```bash
     python scripts/populate_book_index.py
     ```
   - Acceptance criteria:
     - All book chunks have corresponding book_index entries
     - Verify count matches Qdrant collection size

---

## 3.6 Frontend Widget Development

### Task 3.6.1: Initialize Frontend Project

**Subtasks**:

1. **Create frontend directory structure**
   - Files to create:
     - `frontend/src/widget.js`
     - `frontend/src/api-service.js`
     - `frontend/src/text-selection.js`
     - `frontend/src/ui-components.js`
     - `frontend/src/styles.css`
     - `frontend/package.json`
     - `frontend/webpack.config.js`
     - `frontend/README.md`
   - Commands to run:
     ```bash
     mkdir -p frontend/src frontend/tests frontend/build
     cd frontend
     npm init -y
     ```
   - Acceptance criteria:
     - Directory structure created
     - package.json initialized

2. **Install frontend dependencies**
   - Files to modify:
     - `frontend/package.json` - Add dependencies: webpack, webpack-cli, babel-loader, css-loader, style-loader
   - Commands to run:
     ```bash
     npm install --save-dev webpack webpack-cli babel-loader @babel/core @babel/preset-env css-loader style-loader
     ```
   - Acceptance criteria:
     - Dependencies installed

---

### Task 3.6.2: Build Widget UI Components

**Subtasks**:

1. **Create widget main component**
   - Files to create:
     - `frontend/src/widget.js`
   - Content:
     - ChatbotWidget class
     - Constructor: Initialize state (open/closed, messages)
     - render(): Create widget HTML structure
     - toggle(): Open/close widget
     - Append widget to document.body on initialization
   - Acceptance criteria:
     - Widget renders in bottom-right corner
     - Click toggles open/close

2. **Create message display component**
   - Files to create:
     - `frontend/src/ui-components.js`
   - Content:
     - createMessageList(): Generate message list container
     - addMessage(sender, text, citations): Append message to list
     - createInputField(): Generate input field and send button
   - Acceptance criteria:
     - Messages display correctly (user vs assistant)
     - Citations rendered as clickable links

3. **Create loading and error states**
   - Files to modify:
     - `frontend/src/ui-components.js`
   - Content:
     - showLoading(): Display spinner during API call
     - hideLoading(): Hide spinner
     - showError(message): Display error message
   - Acceptance criteria:
     - Loading spinner appears during API call
     - Error messages display correctly

---

### Task 3.6.3: Implement API Service Wrapper

**Subtasks**:

1. **Create API service class**
   - Files to create:
     - `frontend/src/api-service.js`
   - Content:
     - ApiService class
     - query(question, topK, selectionText): Send POST request to /query endpoint
     - Return parsed JSON response
     - Handle fetch errors (network, timeout, HTTP errors)
   - Acceptance criteria:
     - API calls work correctly
     - Errors handled gracefully

---

### Task 3.6.4: Implement Text Selection Capture

**Subtasks**:

1. **Create text selection capture function**
   - Files to create:
     - `frontend/src/text-selection.js`
   - Content:
     - getSelectedText(): Use window.getSelection() to capture highlighted text
     - Return selected text or null if none
   - Acceptance criteria:
     - Highlighted text captured correctly

2. **Integrate selection text into widget**
   - Files to modify:
     - `frontend/src/widget.js`
   - Content:
     - On send button click, check for highlighted text
     - If selection exists, pass as selectionText parameter to API
     - Display badge indicating "Using highlighted text"
   - Acceptance criteria:
     - Selection text sent to backend
     - User notified when selection is used

---

### Task 3.6.5: Style Widget with CSS

**Subtasks**:

1. **Create scoped CSS styles**
   - Files to create:
     - `frontend/src/styles.css`
   - Content:
     - Widget container: fixed position bottom-right, z-index high
     - Widget icon: circular button with chat icon
     - Message list: scrollable container, alternating message colors
     - Input field: bottom-fixed, rounded corners
     - Responsive breakpoints (desktop, tablet, mobile)
   - Acceptance criteria:
     - Widget styled professionally
     - No conflicts with Docusaurus CSS
     - Responsive on all viewports

---

### Task 3.6.6: Build Widget with Webpack

**Subtasks**:

1. **Configure Webpack**
   - Files to create:
     - `frontend/webpack.config.js`
   - Content:
     - Entry: src/widget.js
     - Output: build/widget.min.js
     - Loaders: Babel for JS, CSS loader
     - Mode: production (minified)
   - Acceptance criteria:
     - Webpack configured correctly

2. **Build widget bundle**
   - Commands to run:
     ```bash
     npm run build
     ```
   - Files created:
     - `frontend/build/widget.min.js`
   - Acceptance criteria:
     - Bundle generated successfully
     - File size < 50KB gzipped

---

## 3.7 Docusaurus Integration

### Task 3.7.1: Swizzle Docusaurus Root Component

**Subtasks**:

1. **Swizzle Root.js**
   - Commands to run:
     ```bash
     cd docs
     npm run swizzle @docusaurus/theme-classic Root -- --eject
     ```
   - Files created:
     - `docs/src/theme/Root.js`
   - Acceptance criteria:
     - Root.js created successfully

2. **Inject widget script in Root.js**
   - Files to modify:
     - `docs/src/theme/Root.js`
   - Content:
     - Import useEffect from React
     - In useEffect, inject <script> tag pointing to widget.min.js
     - Script src: {process.env.CHATBOT_WIDGET_URL}/widget.min.js
   - Acceptance criteria:
     - Widget script loaded on all pages

---

### Task 3.7.2: Configure Environment Variables for Docusaurus

**Subtasks**:

1. **Add CHATBOT_API_URL to Docusaurus config**
   - Files to modify:
     - `docs/.env.example`
     - `docs/docusaurus.config.js`
   - Content:
     - .env.example: CHATBOT_API_URL=https://chatbot-api.railway.app
     - docusaurus.config.js: Use customFields to expose env variable to client
   - Acceptance criteria:
     - Widget can access backend API URL

---

### Task 3.7.3: Test Widget on Docusaurus Site

**Subtasks**:

1. **Manual testing on local Docusaurus**
   - Commands to run:
     ```bash
     cd docs
     npm start
     ```
   - Test cases:
     - Widget appears on all chapter pages
     - Widget does not conflict with sidebar or navigation
     - Ask question: "What is ROS 2?"
     - Verify response displays correctly
   - Acceptance criteria:
     - All tests pass

---

## 3.8 Local Development Environment

### Task 3.8.1: Create Dockerfile for Backend

**Subtasks**:

1. **Create Dockerfile**
   - Files to create:
     - `backend/Dockerfile`
   - Content:
     - FROM python:3.10-slim
     - WORKDIR /app
     - COPY requirements.txt, install dependencies
     - COPY src/, migrations/
     - CMD: uvicorn src.main:app --host 0.0.0.0 --port 8000
   - Acceptance criteria:
     - Dockerfile builds successfully

---

### Task 3.8.2: Create Docker Compose for Local Development

**Subtasks**:

1. **Create docker-compose.yml**
   - Files to create:
     - `backend/docker-compose.yml`
   - Content:
     - Service: backend (build from Dockerfile)
     - Service: postgres (optional local DB for testing)
     - Environment variables loaded from .env file
     - Ports: 8000 (backend), 5432 (postgres)
   - Commands to run:
     ```bash
     docker-compose up
     ```
   - Acceptance criteria:
     - Services start successfully
     - Backend accessible at http://localhost:8000

---

### Task 3.8.3: Write Local Development Setup Guide

**Subtasks**:

1. **Create README.md**
   - Files to create:
     - `backend/README.md`
   - Content:
     - Prerequisites (Python 3.10+, Docker, Node.js)
     - Setup steps:
       1. Clone repo
       2. Copy .env.example to .env, fill in credentials
       3. Run docker-compose up
       4. Run migrations: alembic upgrade head
       5. Test: curl http://localhost:8000/health
     - Troubleshooting section
   - Acceptance criteria:
     - New developers can follow README to get started

---

## 3.9 Production Deployment

### Task 3.9.1: Deploy Backend to Hosting Platform

**Subtasks**:

1. **Create deployment configuration (Railway/Render/Fly.io)**
   - Files to create (choose one):
     - `backend/railway.json` (for Railway)
     - `backend/render.yaml` (for Render)
     - `backend/fly.toml` (for Fly.io)
   - Content (example for Railway):
     - Build command: pip install -r requirements.txt
     - Start command: uvicorn src.main:app --host 0.0.0.0 --port $PORT
     - Environment variables: (set in Railway dashboard)
   - Commands to run:
     ```bash
     railway up  # or equivalent for chosen platform
     ```
   - Acceptance criteria:
     - Backend deployed successfully
     - Accessible via HTTPS URL

2. **Configure environment variables in hosting platform**
   - Steps:
     - Navigate to platform dashboard (Railway, Render, etc.)
     - Add environment variables:
       - QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION
       - OPENAI_API_KEY, OPENAI_MODEL
       - NEON_DATABASE_URL
     - Redeploy service
   - Acceptance criteria:
     - All environment variables loaded correctly
     - /query endpoint works in production

---

### Task 3.9.2: Configure CORS for Production

**Subtasks**:

1. **Update CORS whitelist**
   - Files to modify:
     - `backend/src/main.py`
   - Content:
     - Add production Docusaurus domain to CORS allowed origins
     - Example: https://yourbook.com, https://yourbook.github.io
   - Acceptance criteria:
     - Widget can make cross-origin requests from production domain

---

### Task 3.9.3: Set Up Health Check and Monitoring

**Subtasks**:

1. **Test health check endpoint**
   - Commands to run:
     ```bash
     curl https://chatbot-api.railway.app/health
     ```
   - Acceptance criteria:
     - Returns 200 OK with {"status": "ok"}

2. **Configure uptime monitoring (optional)**
   - Tools: UptimeRobot, Pingdom, or platform-native monitoring
   - Steps:
     - Create uptime check for /health endpoint
     - Set alert threshold (e.g., 2 consecutive failures)
   - Acceptance criteria:
     - Monitoring active

---

## 3.10 Testing & Validation

### Task 3.10.1: Write Unit Tests

**Subtasks**:

1. **Unit test Qdrant service**
   - Files to create:
     - `backend/tests/unit/test_qdrant_service.py`
   - Content:
     - Mock Qdrant client
     - Test search function with various inputs
     - Test similarity threshold filtering
     - Test error handling
   - Commands to run:
     ```bash
     pytest tests/unit/test_qdrant_service.py
     ```
   - Acceptance criteria:
     - All tests pass
     - >90% code coverage

2. **Unit test OpenAI service**
   - Files to create:
     - `backend/tests/unit/test_openai_service.py`
   - Content:
     - Mock OpenAI client
     - Test answer generation
     - Test selection_text mode
     - Test rate limit handling
   - Commands to run:
     ```bash
     pytest tests/unit/test_openai_service.py
     ```
   - Acceptance criteria:
     - All tests pass

3. **Unit test grounding service**
   - Files to create:
     - `backend/tests/unit/test_grounding_service.py`
   - Content:
     - Test citation generation
     - Test out-of-scope detection
     - Test refusal message generation
   - Acceptance criteria:
     - All tests pass

4. **Unit test database service**
   - Files to create:
     - `backend/tests/unit/test_database_service.py`
   - Content:
     - Mock database connection
     - Test insert_chat_log
     - Test query_book_index
   - Acceptance criteria:
     - All tests pass

---

### Task 3.10.2: Write Integration Tests

**Subtasks**:

1. **Integration test /query endpoint**
   - Files to create:
     - `backend/tests/integration/test_query_endpoint.py`
   - Content:
     - Test full RAG flow: query → retrieval → generation → response
     - Test with real Qdrant and OpenAI (or staging/mock services)
     - Test selection_text bypass mode
     - Test out-of-scope queries
   - Commands to run:
     ```bash
     pytest tests/integration/test_query_endpoint.py
     ```
   - Acceptance criteria:
     - All tests pass
     - Response format matches QueryResponse model

---

### Task 3.10.3: Write E2E Tests

**Subtasks**:

1. **E2E test widget interaction**
   - Files to create:
     - `frontend/tests/e2e/chatbot-flow.spec.js`
   - Content (Playwright):
     - Open Docusaurus page
     - Click chatbot widget icon
     - Type question: "What is ROS 2?"
     - Click send button
     - Assert response displays within 5 seconds
     - Assert citations include chapter links
   - Commands to run:
     ```bash
     npx playwright test
     ```
   - Acceptance criteria:
     - E2E tests pass on Chrome, Firefox, Safari

---

### Task 3.10.4: Manual Validation Checklist

**Subtasks**:

1. **Create manual testing checklist**
   - Files to create:
     - `specs/002-rag-chatbot-integration/manual-testing-checklist.md`
   - Content:
     - [ ] Ask 100 in-scope questions, verify answers are accurate and grounded
     - [ ] Ask 20 out-of-scope questions, verify refusal messages
     - [ ] Test highlighted text mode on 10 queries
     - [ ] Test error states: API down, rate limit, network error
     - [ ] Test on desktop, tablet, mobile viewports
     - [ ] Verify citations link to correct chapters
     - [ ] Verify chat logs saved to database
     - [ ] Load test: 100 concurrent users
   - Acceptance criteria:
     - All checklist items completed

2. **Grounding validation report**
   - Files to create:
     - `specs/002-rag-chatbot-integration/grounding-validation-report.md`
   - Content:
     - Test 100 queries, document:
       - Question
       - Expected answer (manual review)
       - Actual answer
       - Grounding status (grounded / hallucinated)
       - Citation accuracy (correct / incorrect)
     - Calculate accuracy: (correct answers / total) * 100
   - Acceptance criteria:
     - Accuracy >= 95%
     - Zero hallucinations detected

---

# 4. BACKEND TASKS (Detailed)

All backend tasks are covered in sections 3.1 through 3.5 above. Key highlights:

- **Environment Setup**: Python 3.10+, FastAPI, Pydantic, environment variables
- **API Layer**: /query endpoint with request/response models, CORS, error handling
- **RAG Core**: Qdrant retrieval, OpenAI generation, grounding validation
- **Database**: Neon Postgres with Alembic migrations, chat logging, book index
- **Testing**: Unit, integration, E2E tests for all services

**Critical Path Tasks**:
1. Initialize FastAPI project (Task 3.1.1)
2. Set up Qdrant client and retrieval (Task 3.2)
3. Implement OpenAI answer generation (Task 3.3)
4. Create /query endpoint (Task 3.1.3)
5. Set up Neon database and migrations (Task 3.5)

---

# 5. FRONTEND TASKS (Detailed)

All frontend tasks are covered in sections 3.6 and 3.7 above. Key highlights:

- **Widget UI**: Chat interface with message list, input field, send button
- **API Integration**: HTTP client for /query endpoint
- **Text Selection**: Capture highlighted text from page
- **Styling**: Scoped CSS, responsive design, no conflicts with Docusaurus
- **Build Process**: Webpack bundle (widget.min.js < 50KB gzipped)
- **Docusaurus Integration**: Root.js swizzling, script injection

**Critical Path Tasks**:
1. Initialize frontend project (Task 3.6.1)
2. Build widget UI components (Task 3.6.2)
3. Implement API service wrapper (Task 3.6.3)
4. Style widget with CSS (Task 3.6.5)
5. Integrate into Docusaurus (Task 3.7)

---

# 6. DATABASE TASKS (Detailed)

All database tasks are covered in section 3.5 above. Key highlights:

- **Schema Design**: chat_logs (id, question, answer, timestamp, retrieval_metadata, model_used), book_index (id, qdrant_id, chapter_title, file_path, section_heading)
- **ORM Models**: SQLAlchemy models for ChatLog and BookIndex
- **Migrations**: Alembic migrations for schema versioning
- **Services**: Database service for insert and query operations
- **Population**: Script to populate book_index from Qdrant metadata

**Critical Path Tasks**:
1. Create SQLAlchemy models (Task 3.5.2)
2. Set up Alembic migrations (Task 3.5.3)
3. Create migration files and run migrations (Task 3.5.4)
4. Implement database service functions (Task 3.5.5)
5. Populate book_index table (Task 3.5.6)

---

# 7. DEPLOYMENT TASKS (Detailed)

All deployment tasks are covered in sections 3.8 and 3.9 above. Key highlights:

- **Local Development**: Docker Compose with backend service, optional Postgres
- **Production Deployment**: Railway/Render/Fly.io with environment variables
- **CORS Configuration**: Whitelist production Docusaurus domain
- **HTTPS**: Enforced in production
- **Monitoring**: Health check endpoint, optional uptime monitoring

**Critical Path Tasks**:
1. Create Dockerfile (Task 3.8.1)
2. Create docker-compose.yml (Task 3.8.2)
3. Deploy to hosting platform (Task 3.9.1)
4. Configure environment variables (Task 3.9.1 sub-task 2)
5. Configure CORS (Task 3.9.2)

---

# 8. TESTING STRATEGY

## 8.1 Unit Testing

**Scope**: Test individual functions and classes in isolation.

**Tools**: Pytest, Pytest-asyncio, unittest.mock

**Coverage Target**: >90% code coverage for all services

**Key Test Suites**:
- `test_qdrant_service.py`: Mock Qdrant client, test search, embedding, error handling
- `test_openai_service.py`: Mock OpenAI client, test answer generation, rate limits
- `test_grounding_service.py`: Test citation extraction, out-of-scope detection
- `test_database_service.py`: Mock database, test insert and query operations

**Mocking Strategy**:
- Mock external APIs (Qdrant, OpenAI, Neon) to avoid real API calls
- Use `unittest.mock.patch` or `pytest-mock`
- Verify function behavior with different inputs (valid, invalid, edge cases)

**Example Test Case** (test_qdrant_service.py):
```python
def test_search_returns_top_k_chunks(mock_qdrant_client):
    # Mock Qdrant search response
    mock_qdrant_client.search.return_value = [
        {"id": "chunk1", "score": 0.9, "payload": {"text": "ROS 2 is..."}},
        {"id": "chunk2", "score": 0.85, "payload": {"text": "Nodes in ROS 2..."}}
    ]

    service = QdrantService()
    result = service.search("What is ROS 2?", top_k=2)

    assert len(result) == 2
    assert result[0].similarity_score == 0.9
```

---

## 8.2 Integration Testing

**Scope**: Test interactions between components (API endpoint + services + database).

**Tools**: Pytest, Httpx (async HTTP client), TestClient (FastAPI)

**Coverage Target**: All API endpoints tested with real service integrations

**Key Test Suites**:
- `test_query_endpoint.py`: Test /query endpoint with full RAG flow
- `test_full_rag_flow.py`: Test retrieval → generation → logging pipeline

**Test Environment**:
- Use staging Qdrant collection or dedicated test collection
- Use OpenAI API with small test budget or mock service
- Use local Postgres or Neon test database

**Example Test Case** (test_query_endpoint.py):
```python
def test_query_endpoint_returns_valid_response(test_client):
    response = test_client.post("/query", json={
        "q": "What is ROS 2?",
        "top_k": 5
    })

    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert len(data["sources"]) > 0
```

---

## 8.3 End-to-End Testing

**Scope**: Test complete user workflows from widget interaction to backend response.

**Tools**: Playwright (browser automation), Jest (widget unit tests)

**Coverage Target**: All user stories from spec.md tested in real browser

**Key Test Scenarios**:
1. User opens widget, asks question, receives answer with citations
2. User highlights text, asks question, receives context-specific answer
3. User asks out-of-scope question, receives refusal message
4. User experiences error states (API down, rate limit)

**Example E2E Test** (chatbot-flow.spec.js):
```javascript
test('user can ask question and receive answer', async ({ page }) => {
  await page.goto('http://localhost:3000/docs/ros2/introduction');

  // Click widget icon
  await page.click('#chatbot-widget-icon');

  // Type question
  await page.fill('#chatbot-input', 'What is ROS 2?');
  await page.click('#chatbot-send');

  // Wait for response
  await page.waitForSelector('.chatbot-message.assistant', { timeout: 5000 });

  // Assert answer displayed
  const answer = await page.textContent('.chatbot-message.assistant');
  expect(answer).toContain('ROS 2');
});
```

---

## 8.4 Mocking Strategy for External APIs

### Mocking Qdrant

**Library**: `unittest.mock.patch`

**Mock Target**: `qdrant_client.QdrantClient.search`

**Mock Response**:
```python
{
    "id": "chunk_123",
    "score": 0.89,
    "payload": {
        "text": "ROS 2 is a robotics middleware...",
        "chapter": "Module 1: ROS 2 Fundamentals",
        "file_path": "/docs/ros2/introduction"
    }
}
```

---

### Mocking OpenAI

**Library**: `unittest.mock.patch`

**Mock Target**: `openai.ChatCompletion.create`

**Mock Response**:
```python
{
    "choices": [{
        "message": {
            "content": "ROS 2 is a robotics middleware that provides libraries and tools for building robot applications. (Source: Module 1: ROS 2 Fundamentals)"
        }
    }],
    "usage": {
        "prompt_tokens": 150,
        "completion_tokens": 50
    }
}
```

---

## 8.5 Testing RAG Grounding

**Objective**: Ensure chatbot never hallucinates or uses external knowledge.

**Approach**:
1. Create test dataset of 100 in-scope questions with expected answers
2. Create test dataset of 20 out-of-scope questions
3. Query chatbot for each question
4. Manual review: Does answer match retrieved context?
5. Flag hallucinations (answer includes facts not in context)

**Example Test**:
- Question: "What is the capital of France?"
- Expected: Refusal message (out of scope)
- Actual: "I can only answer questions based on the Physical AI textbook..."
- Status: ✅ PASS

---

## 8.6 Testing selection_text Override

**Objective**: Verify that when `selection_text` is provided, Qdrant retrieval is bypassed.

**Approach**:
1. Highlight text on page: "ROS 2 nodes are independent processes..."
2. Ask question: "What are nodes?"
3. Verify backend receives `selection_text` parameter
4. Verify Qdrant search is NOT called (mock Qdrant and assert no calls)
5. Verify answer is generated using only selection_text as context

**Example Test**:
```python
def test_selection_text_bypasses_qdrant(mock_qdrant, test_client):
    response = test_client.post("/query", json={
        "q": "What are nodes?",
        "selection_text": "ROS 2 nodes are independent processes that perform computation."
    })

    # Assert Qdrant not called
    mock_qdrant.search.assert_not_called()

    # Assert answer generated from selection_text
    assert response.status_code == 200
    data = response.json()
    assert "independent processes" in data["answer"]
```

---

## 8.7 End-to-End Chatbot Test Plan

**Test Cases**:

1. **Happy Path - Basic Query**
   - Open widget
   - Ask: "What is Physical AI?"
   - Assert: Answer mentions embodied intelligence, citations present
   - Time: <5 seconds

2. **Happy Path - Selection Mode**
   - Highlight text: "Physical AI combines perception, reasoning, and action..."
   - Ask: "Explain this"
   - Assert: Answer based on highlighted text, no Qdrant retrieval
   - Time: <3 seconds

3. **Out of Scope Query**
   - Ask: "What is the weather today?"
   - Assert: Refusal message displayed
   - Time: <5 seconds

4. **Error Handling - API Down**
   - Simulate backend down (disconnect network)
   - Ask: "What is ROS 2?"
   - Assert: Error message "Unable to connect to chatbot service. Please try again later."
   - Time: <5 seconds

5. **Error Handling - Rate Limit**
   - Simulate OpenAI rate limit (mock 429 response)
   - Ask: "What is ROS 2?"
   - Assert: Error message "Service temporarily unavailable. Please try again in a moment."
   - Time: <5 seconds

6. **Citation Links**
   - Ask: "What is URDF?"
   - Click citation link in answer
   - Assert: Browser navigates to /docs/ros2/urdf-robot-modeling chapter
   - Time: <5 seconds

7. **Conversation History**
   - Ask 3 questions
   - Scroll to top of widget
   - Assert: All 3 Q&A pairs displayed in order
   - Time: Immediate

8. **Responsive Design**
   - Open widget on desktop (1920px)
   - Open widget on tablet (768px)
   - Open widget on mobile (375px)
   - Assert: Widget renders correctly on all viewports

---

# 9. ACCEPTANCE CRITERIA

## 9.1 Backend Acceptance Criteria

✅ **Backend Working**:
- [ ] FastAPI server starts without errors
- [ ] /health endpoint returns 200 OK
- [ ] /query endpoint accepts POST requests with valid JSON body
- [ ] /query endpoint returns QueryResponse with answer, sources, confidence
- [ ] CORS configured to allow requests from Docusaurus domain

✅ **Retrieval Accuracy**:
- [ ] Qdrant search retrieves top_k relevant chunks for in-scope queries
- [ ] Similarity threshold (0.6) filters low-relevance chunks
- [ ] Out-of-scope queries (low similarity) return zero chunks
- [ ] Retrieved chunks include metadata: qdrant_id, chunk_text, similarity_score

✅ **Answer Generation**:
- [ ] OpenAI generates answers grounded in retrieved context
- [ ] Answers include citations to source chapters
- [ ] Out-of-scope queries trigger refusal messages
- [ ] selection_text mode bypasses Qdrant retrieval

✅ **Database Logging**:
- [ ] All queries logged to chat_logs table
- [ ] book_index table populated with Qdrant ID → chapter mappings
- [ ] Database queries execute without errors

---

## 9.2 Frontend Acceptance Criteria

✅ **Widget Integration**:
- [ ] Widget appears on all Docusaurus chapter pages
- [ ] Widget icon visible in bottom-right corner
- [ ] Widget opens/closes on click
- [ ] Widget loads within 2 seconds of page load

✅ **UI Functionality**:
- [ ] User can type question in input field (max 500 chars)
- [ ] Send button triggers API call
- [ ] Loading spinner displays during API call
- [ ] Answer displays in conversation history within 5 seconds
- [ ] Citations rendered as clickable links

✅ **Text Selection**:
- [ ] User can highlight text on page
- [ ] Highlighted text passed to backend as selection_text
- [ ] Badge displays "Using highlighted text" when selection active

✅ **Error Handling**:
- [ ] Error message displays when API is unreachable
- [ ] Error message displays when rate limit exceeded
- [ ] Error message displays for network errors

---

## 9.3 Grounding Behavior Acceptance Criteria

✅ **Full Grounding**:
- [ ] 100 in-scope test queries: 95%+ accuracy (answers match retrieved context)
- [ ] 20 out-of-scope test queries: 100% refusal rate (no answers from external knowledge)
- [ ] Zero hallucinations detected in validation dataset
- [ ] All answers cite source chapters

✅ **Citation Accuracy**:
- [ ] 50 random answers tested: 100% citation correctness (links navigate to correct chapters)
- [ ] Citations include chapter title and file path
- [ ] Clicking citation scrolls to relevant section in book

---

## 9.4 Performance Acceptance Criteria

✅ **Latency**:
- [ ] p95 response latency <5 seconds for /query endpoint
- [ ] p50 response latency <3 seconds
- [ ] Widget loads within 2 seconds of page load

✅ **Concurrency**:
- [ ] System handles 100 concurrent requests without errors
- [ ] Response times remain stable under load

✅ **Resource Usage**:
- [ ] Widget JavaScript bundle <50KB gzipped
- [ ] Backend memory usage <512MB under normal load

---

## 9.5 Deployment Acceptance Criteria

✅ **Production Deployment**:
- [ ] Backend deployed to hosting platform (Railway/Render/Fly.io)
- [ ] HTTPS enabled
- [ ] Environment variables configured correctly
- [ ] Health check endpoint accessible
- [ ] CORS whitelist includes production domain

✅ **Local Development**:
- [ ] Docker Compose starts all services successfully
- [ ] README setup instructions work for new developers
- [ ] .env.example file includes all required variables

---

# 10. DEPENDENCIES GRAPH

## 10.1 Critical Path (Must Complete in Order)

```
[Milestone 1: Environment Setup]
        ↓
[Milestone 2: Database Schema] ← (blocks logging and citations)
        ↓
[Milestone 3: Retrieval Layer] ← (blocks answer generation)
        ↓
[Milestone 4: Generation Layer] ← (blocks /query endpoint)
        ↓
[Milestone 5: /query Endpoint] ← (blocks frontend integration)
        ↓
[Milestone 6: Widget UI] ← (blocks Docusaurus integration)
        ↓
[Milestone 7: Widget API Integration] ← (blocks Docusaurus integration)
        ↓
[Milestone 8: Docusaurus Integration] ← (blocks production deployment)
        ↓
[Milestone 9: Production Deployment] ← (blocks final validation)
        ↓
[Milestone 10: Testing & Validation]
```

**Critical Path Duration**: 10 milestones

---

## 10.2 Parallel Work Opportunities

**Can be done in parallel with Milestone 3-4**:
- Frontend project initialization (Milestone 6 prep)
- Local development Docker setup (Milestone 8)
- Unit test scaffolding (Milestone 10)

**Can be done in parallel with Milestone 5**:
- Widget UI development (Milestone 6)
- Deployment configuration files (Milestone 9 prep)

**Can be done in parallel with Milestone 8**:
- E2E test setup (Milestone 10)
- Manual testing checklist creation (Milestone 10)

---

## 10.3 Task Dependencies (Detailed)

### Backend Dependencies

| Task | Depends On | Blocks |
|------|------------|--------|
| 3.1.1 Initialize FastAPI | None | 3.1.2, 3.1.3, 3.1.4 |
| 3.2.1 Qdrant Client Setup | 3.1.1 | 3.2.2, 3.2.3 |
| 3.2.3 Semantic Search | 3.2.2 | 3.4.1, 3.1.3 |
| 3.3.1 OpenAI Service | 3.1.1 | 3.3.3 |
| 3.3.3 Answer Generation | 3.3.1, 3.2.3 | 3.1.3 |
| 3.4.2 Citation Generation | 3.5.2 (book_index model) | 3.1.3 |
| 3.5.4 Run Migrations | 3.5.2, 3.5.3 | 3.5.5, 3.4.2 |
| 3.1.3 /query Endpoint | 3.2.3, 3.3.3, 3.4.2, 3.5.5 | 3.6.3, 3.10.2 |

---

### Frontend Dependencies

| Task | Depends On | Blocks |
|------|------------|--------|
| 3.6.1 Initialize Frontend | None | 3.6.2, 3.6.3, 3.6.5 |
| 3.6.3 API Service | 3.6.1, 3.1.3 (/query available) | 3.6.2 |
| 3.6.4 Text Selection | 3.6.1 | 3.6.2 |
| 3.6.6 Webpack Build | 3.6.1, 3.6.2, 3.6.5 | 3.7.1 |
| 3.7.1 Docusaurus Integration | 3.6.6, Feature 001 (Docusaurus site) | 3.9.1 |

---

### Testing Dependencies

| Task | Depends On | Blocks |
|------|------------|--------|
| 3.10.1 Unit Tests | Corresponding service tasks | 3.10.4 (validation) |
| 3.10.2 Integration Tests | 3.1.3 (/query endpoint) | 3.10.4 |
| 3.10.3 E2E Tests | 3.7.1 (widget in Docusaurus) | 3.10.4 |
| 3.10.4 Manual Validation | 3.9.1 (production deployment) | None (final gate) |

---

## 10.4 Blocking Tasks (High Risk)

**⚠️ Tasks that block multiple downstream tasks**:

1. **Milestone 2 (Database Schema)**: Blocks logging (3.5.5), citations (3.4.2), /query endpoint (3.1.3)
2. **Milestone 3 (Retrieval Layer)**: Blocks answer generation (3.3.3), /query endpoint (3.1.3)
3. **Milestone 5 (/query Endpoint)**: Blocks frontend API integration (3.6.3), E2E tests (3.10.3)
4. **Milestone 7 (Widget API Integration)**: Blocks Docusaurus integration (3.7.1)

**Risk Mitigation**:
- Prioritize blocking tasks in sprint planning
- Implement mocks/stubs to unblock downstream work
- Run daily integration checks to catch blockers early

---

## 10.5 Recommended Implementation Order

**Sprint 1: Foundation (Milestones 1-2)**
- Initialize backend and frontend projects
- Set up database schema and migrations
- Validate connections to Qdrant, OpenAI, Neon

**Sprint 2: RAG Core (Milestones 3-4)**
- Implement Qdrant retrieval service
- Implement OpenAI answer generation
- Implement grounding and citation service
- Write unit tests

**Sprint 3: API & Widget (Milestones 5-6)**
- Implement /query endpoint
- Build widget UI components
- Write integration tests

**Sprint 4: Integration (Milestones 7-8)**
- Integrate widget with backend API
- Implement text selection mode
- Integrate widget into Docusaurus

**Sprint 5: Deployment & Validation (Milestones 9-10)**
- Deploy backend to production
- Run E2E tests
- Complete manual validation checklist
- Grounding validation report

---

# APPENDICES

## Appendix A: Environment Variables Reference

| Variable | Description | Example | Required |
|----------|-------------|---------|----------|
| QDRANT_URL | Qdrant Cloud cluster URL | https://xyz.qdrant.io | Yes |
| QDRANT_API_KEY | Qdrant API key | qdr_abc123... | Yes |
| QDRANT_COLLECTION | Collection name | physical_ai_textbook | Yes |
| OPENAI_API_KEY | OpenAI API key | sk-proj-abc123... | Yes |
| OPENAI_MODEL | Model name | gpt-4o-mini | Yes |
| NEON_DATABASE_URL | Neon Postgres connection string | postgresql://user:pass@host/db | Yes |
| CHATBOT_API_URL | Backend API URL (frontend) | https://chatbot-api.railway.app | Yes |

---

## Appendix B: API Contract Specification

### POST /query

**Endpoint**: `/query`

**Method**: POST

**Request Body**:
```json
{
  "q": "What is ROS 2?",
  "top_k": 5,
  "selection_text": "ROS 2 is a robotics middleware..." (optional)
}
```

**Response Body** (200 OK):
```json
{
  "answer": "ROS 2 is a robotics middleware that provides libraries and tools for building robot applications. It enables communication between distributed processes using nodes, topics, services, and actions. (Source: Module 1: ROS 2 Fundamentals)",
  "sources": [
    {
      "chapter": "Module 1: ROS 2 Fundamentals",
      "file_path": "/docs/ros2/introduction",
      "chunk_text": "ROS 2 is a robotics middleware..."
    }
  ],
  "confidence": 0.89
}
```

**Error Responses**:
- 422 Unprocessable Entity: Invalid request body (query too long, top_k out of range)
- 429 Too Many Requests: OpenAI rate limit exceeded
- 503 Service Unavailable: Qdrant or OpenAI API unreachable

---

## Appendix C: Database Schemas

### chat_logs Table

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | INTEGER | PRIMARY KEY, AUTOINCREMENT | Unique log ID |
| question | TEXT | NOT NULL | User query |
| answer | TEXT | NOT NULL | Chatbot response |
| timestamp | TIMESTAMP | DEFAULT NOW() | Query timestamp |
| retrieval_metadata | JSON | NULL | Qdrant chunk IDs, similarity scores |
| model_used | VARCHAR(50) | NOT NULL | OpenAI model (e.g., gpt-4o-mini) |

### book_index Table

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | INTEGER | PRIMARY KEY, AUTOINCREMENT | Unique index ID |
| qdrant_id | VARCHAR(255) | UNIQUE, NOT NULL | Qdrant chunk ID |
| chapter_title | VARCHAR(255) | NOT NULL | Chapter name (e.g., "Module 1: ROS 2 Fundamentals") |
| file_path | VARCHAR(500) | NOT NULL | Docusaurus page path (e.g., /docs/ros2/introduction) |
| section_heading | VARCHAR(255) | NULL | Section within chapter (optional) |

---

## Appendix D: Widget Embedding Example

**Docusaurus Root.js Integration**:

```javascript
// docs/src/theme/Root.js
import React, { useEffect } from 'react';

export default function Root({ children }) {
  useEffect(() => {
    // Inject chatbot widget script
    const script = document.createElement('script');
    script.src = 'https://your-cdn.com/widget.min.js';
    script.async = true;
    document.body.appendChild(script);

    return () => {
      document.body.removeChild(script);
    };
  }, []);

  return <>{children}</>;
}
```

---

## Appendix E: Grounding Prompt Template

```
You are a helpful assistant answering questions about the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. Answer ONLY using the provided context below. Do NOT use external knowledge.
2. If the context does not contain the answer, respond with: "I couldn't find information on this topic in the textbook."
3. Cite your sources by referencing the chapter name in your answer (e.g., "According to Module 1: ROS 2 Fundamentals, ...").
4. Be concise and accurate.
5. If the question is completely unrelated to the textbook content (e.g., current events, general knowledge), respond with: "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content."

Context:
{context}

Question: {question}

Answer:
```

---

**END OF IMPLEMENTATION PLAN**
