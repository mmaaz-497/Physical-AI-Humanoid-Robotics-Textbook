# Tasks: Integrated RAG Chatbot for Physical AI Book

**Input**: Design documents from `/specs/002-rag-chatbot-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are OPTIONAL and NOT included in this task list per specification requirements.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/src/`, `frontend/src/`
- Backend: Python 3.10+, FastAPI, Qdrant Client, OpenAI SDK, SQLAlchemy
- Frontend: JavaScript ES6+, Webpack

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend directory structure: backend/src/, backend/tests/, backend/migrations/
- [ ] T002 Create frontend directory structure: frontend/src/, frontend/tests/, frontend/build/
- [ ] T003 [P] Initialize backend Python project with pyproject.toml and requirements.txt
- [ ] T004 [P] Initialize frontend Node.js project with package.json
- [ ] T005 [P] Create backend/.env.example with all required environment variables (QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION, OPENAI_API_KEY, OPENAI_MODEL, NEON_DATABASE_URL)
- [ ] T006 [P] Create backend/README.md with local development setup instructions
- [ ] T007 [P] Create frontend/README.md with widget build and integration instructions

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T008 Install FastAPI and core dependencies in backend/requirements.txt (fastapi, uvicorn, pydantic, pydantic-settings, python-dotenv)
- [ ] T009 Create backend/src/config.py for environment variable loading using Pydantic BaseSettings
- [ ] T010 Create backend/src/main.py with FastAPI app initialization, CORS middleware, and health check endpoint (/health)
- [ ] T011 [P] Create backend/src/utils/logging.py for structured JSON logging configuration
- [ ] T012 [P] Create backend/src/utils/exceptions.py with custom exception classes (QdrantConnectionError, OpenAIRateLimitError, OutOfScopeQueryError, DatabaseError)
- [ ] T013 Add global exception handlers in backend/src/main.py for all custom exceptions
- [ ] T014 Install Qdrant client library in backend/requirements.txt (qdrant-client>=1.7.0)
- [ ] T015 Install OpenAI library in backend/requirements.txt (openai>=1.10.0)
- [ ] T016 Install SQLAlchemy and Alembic in backend/requirements.txt (sqlalchemy>=2.0, psycopg2-binary>=2.9, alembic>=1.12)

### Database Foundation

- [ ] T017 Create backend/src/database.py with SQLAlchemy engine, session factory, and Base for declarative models
- [ ] T018 Initialize Alembic for database migrations: run `alembic init migrations` in backend/
- [ ] T019 Configure Alembic in backend/alembic.ini and backend/migrations/env.py to use NEON_DATABASE_URL
- [ ] T020 [P] Create backend/src/models/database.py with ChatLog SQLAlchemy model (id, question, answer, timestamp, retrieval_metadata JSON, model_used)
- [ ] T021 [P] Create backend/src/models/database.py with BookIndex SQLAlchemy model (id, qdrant_id unique, chapter_title, file_path, section_heading nullable)
- [ ] T022 Generate Alembic migration for chat_logs table: `alembic revision --autogenerate -m "create_chat_logs"`
- [ ] T023 Generate Alembic migration for book_index table: `alembic revision --autogenerate -m "create_book_index"`
- [ ] T024 Run database migrations: `alembic upgrade head`
- [ ] T025 Create backend/src/services/database_service.py with DatabaseService class (insert_chat_log, query_book_index methods)

### Frontend Foundation

- [ ] T026 Install frontend dependencies in frontend/package.json (webpack, webpack-cli, babel-loader, @babel/core, @babel/preset-env, css-loader, style-loader)
- [ ] T027 Create frontend/webpack.config.js for building widget.min.js bundle with CSS support

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 + 6 - Ask Questions with Zero Hallucinations (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions about the Physical AI textbook and receive accurate, grounded answers with citations. System must NEVER hallucinate or use external knowledge (US6 critical constraint).

**Independent Test**: Open any Docusaurus chapter page, type "What is ROS 2?", verify chatbot returns accurate answer citing specific book sections. Ask "What is the capital of France?" and verify refusal message.

### Backend RAG Core (US1 + US6)

- [ ] T028 [P] [US1] Create backend/src/models/request.py with QueryRequest Pydantic model (q: str max_length=500, top_k: int default=5 ge=1 le=10, selection_text: Optional[str] max_length=5000)
- [ ] T029 [P] [US1] Create backend/src/models/response.py with CitationSource and QueryResponse Pydantic models (answer, sources List[CitationSource], confidence float 0-1)
- [ ] T030 [P] [US1] Create backend/src/models/retrieval.py with RetrievedChunk dataclass (qdrant_id, chunk_text, similarity_score, metadata)
- [ ] T031 [US1] Create backend/src/services/qdrant_service.py with QdrantService class (__init__ connects to Qdrant Cloud using config)
- [ ] T032 [US1] Implement generate_query_embedding method in backend/src/services/qdrant_service.py using OpenAI text-embedding-3-small (returns 1536-dim vector)
- [ ] T033 [US1] Implement search method in backend/src/services/qdrant_service.py (query: str, top_k: int, similarity_threshold: float=0.6) returns List[RetrievedChunk]
- [ ] T034 [US1] Add error handling to backend/src/services/qdrant_service.py for timeouts and connection failures with retry logic (max 3 retries, exponential backoff)
- [ ] T035 [US1] Create backend/src/services/openai_service.py with OpenAIService class (__init__ connects to OpenAI API using config)
- [ ] T036 [US6] Define GROUNDING_SYSTEM_PROMPT constant in backend/src/services/openai_service.py enforcing answer grounding rules and zero hallucinations
- [ ] T037 [US1] Implement generate_answer method in backend/src/services/openai_service.py (question: str, context_chunks: List[RetrievedChunk]) returns str using gpt-4o-mini
- [ ] T038 [US1] Add rate limit handling and retry logic to backend/src/services/openai_service.py for OpenAI API errors
- [ ] T039 [US1] Add token usage tracking and logging in backend/src/services/openai_service.py
- [ ] T040 [US6] Create backend/src/services/grounding_service.py with GroundingService class
- [ ] T041 [US6] Implement is_out_of_scope method in backend/src/services/grounding_service.py (checks if all chunks below similarity threshold)
- [ ] T042 [US6] Implement generate_refusal_message method in backend/src/services/grounding_service.py (returns user-friendly refusal for out-of-scope queries)

### Backend API Endpoint (US1)

- [ ] T043 [US1] Create backend/src/routers/query.py with POST /query endpoint accepting QueryRequest and returning QueryResponse
- [ ] T044 [US1] Implement RAG orchestration logic in backend/src/routers/query.py: validate request → call qdrant_service.search → call openai_service.generate_answer → build QueryResponse
- [ ] T045 [US6] Add out-of-scope detection in backend/src/routers/query.py: if is_out_of_scope returns True, return refusal message instead of generating answer
- [ ] T046 [US1] Register query router in backend/src/main.py with app.include_router(query_router)

**Checkpoint**: Backend API MVP ready - /query endpoint functional with zero hallucinations enforced

---

## Phase 4: User Story 3 - Review Past Interactions (Priority: P3)

**Goal**: Log all chat interactions to Neon database for analytics, debugging, and review by educators.

**Independent Test**: Ask 5 different questions via /query endpoint, then query Neon chat_logs table and verify all 5 Q&A pairs are logged with timestamps and metadata.

### Database Logging (US3)

- [ ] T047 [US3] Add chat logging call in backend/src/routers/query.py: after generating answer, call database_service.insert_chat_log with question, answer, retrieval_metadata (Qdrant chunk IDs, similarity scores), model_used
- [ ] T048 [US3] Handle database logging errors gracefully in backend/src/routers/query.py (log errors but don't fail the request)

**Checkpoint**: All queries logged to database - educators can review chat history

---

## Phase 5: User Story 4 - Navigate to Specific Chapters (Priority: P4)

**Goal**: Generate clickable citations linking answers to specific book chapters/sections.

**Independent Test**: Ask "What is Physical AI?", verify response includes citation like "[Source: Introduction to Physical AI](/docs/introduction/what-is-physical-ai)" and clicking link navigates to correct chapter.

### Citation Generation (US4)

- [ ] T049 [US4] Populate backend book_index table: create backend/scripts/populate_book_index.py script to map Qdrant IDs → chapter_title → file_path → section_heading
- [ ] T050 [US4] Run population script to insert all book chunk mappings into book_index table
- [ ] T051 [US4] Implement generate_citations method in backend/src/services/grounding_service.py (context_chunks: List[RetrievedChunk], book_index_service: DatabaseService) returns List[CitationSource]
- [ ] T052 [US4] For each retrieved chunk in generate_citations, lookup chapter info from book_index using qdrant_id and build CitationSource object
- [ ] T053 [US4] Update backend/src/routers/query.py to call grounding_service.generate_citations and populate sources field in QueryResponse

**Checkpoint**: Citations working - answers link to source chapters

---

## Phase 6: User Story 2 - Highlighted Text Bypass Mode (Priority: P2)

**Goal**: When user highlights text on page, bypass Qdrant retrieval and use only the highlighted text as context for faster, more precise answers.

**Independent Test**: Highlight "ROS 2 nodes are independent processes..." on a chapter page, ask "What are nodes?", verify answer uses ONLY highlighted text (no Qdrant retrieval) and mentions "independent processes".

### Selection Text Mode (US2)

- [ ] T054 [US2] Implement generate_answer_from_selection method in backend/src/services/openai_service.py (question: str, selection_text: str) returns str using same grounding prompt
- [ ] T055 [US2] Update backend/src/routers/query.py /query endpoint: if selection_text is provided in QueryRequest, skip Qdrant search and call generate_answer_from_selection instead
- [ ] T056 [US2] Update QueryResponse metadata in backend/src/routers/query.py to indicate when selection_text mode was used (add flag or log message)

**Checkpoint**: Selection text bypass mode functional - faster answers for highlighted content

---

## Phase 7: User Story 5 - Embed Chatbot Widget (Priority: P5)

**Goal**: Create universal JavaScript widget that embeds in Docusaurus site, displays chat UI, captures highlighted text, and calls /query endpoint.

**Independent Test**: Open any Docusaurus chapter page, see widget icon in bottom-right corner, click to open, type question, verify response displays within 5 seconds.

### Frontend Widget Development (US5)

- [ ] T057 [P] [US5] Create frontend/src/widget.js with ChatbotWidget class (constructor, render, toggle open/close methods)
- [ ] T058 [P] [US5] Create frontend/src/ui-components.js with createMessageList, addMessage (renders user/assistant messages with citations), createInputField functions
- [ ] T059 [P] [US5] Create frontend/src/api-service.js with ApiService class (query method sends POST to /query endpoint, returns parsed JSON)
- [ ] T060 [P] [US5] Create frontend/src/text-selection.js with getSelectedText function using window.getSelection()
- [ ] T061 [US5] Implement widget render in frontend/src/widget.js: create floating icon in bottom-right corner, create chat panel (message list + input field + send button)
- [ ] T062 [US5] Implement toggle functionality in frontend/src/widget.js: clicking icon opens/closes chat panel
- [ ] T063 [US5] Implement send message handler in frontend/src/widget.js: on send button click, get query from input, call api-service.query, display response in message list
- [ ] T064 [US5] Integrate text selection in frontend/src/widget.js: on send, check for highlighted text using getSelectedText, pass as selection_text parameter if present
- [ ] T065 [US5] Add loading state in frontend/src/widget.js: show spinner during API call, hide when response received
- [ ] T066 [US5] Add error handling in frontend/src/widget.js: display user-friendly error messages for API failures (network error, API down, rate limit)
- [ ] T067 [US5] Create frontend/src/styles.css with scoped widget styles (fixed position bottom-right, z-index high, responsive for desktop/tablet/mobile, no conflicts with Docusaurus CSS)
- [ ] T068 [US5] Build widget bundle: run `npm run build` to generate frontend/build/widget.min.js (<50KB gzipped)
- [ ] T069 [US5] Create docs/src/theme/Root.js (swizzle Docusaurus Root component) to inject widget script on all pages
- [ ] T070 [US5] Add widget script injection in docs/src/theme/Root.js using useEffect to load widget.min.js
- [ ] T071 [US5] Configure CHATBOT_API_URL environment variable in Docusaurus docs/.env and expose via customFields in docusaurus.config.js

**Checkpoint**: Widget embedded in Docusaurus - students can ask questions from any chapter page

---

## Phase 8: Deployment & Production Readiness

**Purpose**: Deploy backend to production, configure environment variables, enable HTTPS and CORS

- [ ] T072 Create backend/Dockerfile with Python 3.10-slim base image, install dependencies, copy src/ and migrations/, CMD uvicorn
- [ ] T073 [P] Create backend/docker-compose.yml for local development with backend service and optional local Postgres
- [ ] T074 Create deployment configuration file for hosting platform (backend/railway.json OR backend/render.yaml OR backend/fly.toml depending on chosen platform)
- [ ] T075 Deploy backend to hosting platform (Railway/Render/Fly.io) with environment variables configured
- [ ] T076 Verify backend deployment: test GET /health endpoint returns 200 OK from production URL
- [ ] T077 Update CORS allowed origins in backend/src/main.py to whitelist production Docusaurus domain (https://yourbook.com or https://yourbook.github.io)
- [ ] T078 Test /query endpoint from production with CORS enabled
- [ ] T079 Update frontend widget API URL in docs/src/theme/Root.js to point to production backend URL

**Checkpoint**: Production deployment complete - chatbot accessible from live Docusaurus site

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T080 [P] Update backend/README.md with production deployment instructions and environment variable documentation
- [ ] T081 [P] Update frontend/README.md with widget embedding instructions for non-Docusaurus sites
- [ ] T082 [P] Add inline code comments to complex functions in backend/src/services/
- [ ] T083 [P] Format all backend Python code with Black: `black backend/src/`
- [ ] T084 [P] Format all frontend JavaScript code with Prettier: `prettier --write frontend/src/`
- [ ] T085 Validate all environment variables are documented in backend/.env.example
- [ ] T086 Manual validation: Ask 20 in-scope questions and verify 95%+ accuracy
- [ ] T087 Manual validation: Ask 5 out-of-scope questions and verify 100% refusal rate
- [ ] T088 Manual validation: Test widget on desktop (1920px), tablet (768px), mobile (375px) viewports
- [ ] T089 Load test: Verify backend handles 100 concurrent requests without errors
- [ ] T090 Verify widget JavaScript bundle size is <50KB gzipped

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - **US1+US6 (Phase 3 - P1)**: MVP - Can start after Foundational ✅ HIGHEST PRIORITY
  - **US3 (Phase 4 - P3)**: Can start after Foundational, but recommended after US1 MVP working
  - **US4 (Phase 5 - P4)**: Depends on US1 working (needs retrieval context), requires book_index populated
  - **US2 (Phase 6 - P2)**: Depends on US1 backend working (reuses OpenAI service), independent of other stories
  - **US5 (Phase 7 - P5)**: Depends on US1 backend API working, integrates all previous features
- **Deployment (Phase 8)**: Depends on US1 MVP minimum, recommended after US1-US5 complete
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1+6 (P1)**: Can start after Foundational (Phase 2) - **NO dependencies on other stories** - This is the MVP
- **User Story 3 (P3)**: Can start after Foundational, but logically after US1 works (needs something to log)
- **User Story 4 (P4)**: **Depends on US1** retrieval working, needs book_index table populated
- **User Story 2 (P2)**: **Depends on US1** OpenAI service existing, but otherwise independent
- **User Story 5 (P5)**: **Depends on US1** backend API working (/query endpoint must exist)

### Within Each User Story

- Pydantic models before services
- Services before routers/endpoints
- Foundation services (Qdrant, OpenAI, Database) before grounding service
- Backend API before frontend widget
- Core implementation before error handling
- Story complete before moving to next priority

### Parallel Opportunities

**Phase 1 (Setup)**: All tasks T001-T007 can run in parallel

**Phase 2 (Foundational)**:
- T011 (logging.py) + T012 (exceptions.py) can run in parallel
- T020 (ChatLog model) + T021 (BookIndex model) can run in parallel

**Phase 3 (US1+US6)**:
- T028 (request.py) + T029 (response.py) + T030 (retrieval.py) can run in parallel
- After Qdrant and OpenAI services exist: T040-T042 grounding service can run in parallel with endpoint work

**Phase 7 (US5)**:
- T057 (widget.js) + T058 (ui-components.js) + T059 (api-service.js) + T060 (text-selection.js) can run in parallel
- T067 (styles.css) can run in parallel with widget logic

**Phase 8 (Deployment)**:
- T072 (Dockerfile) + T073 (docker-compose.yml) can run in parallel

**Phase 9 (Polish)**:
- T080-T084 documentation and formatting tasks can run in parallel
- T086-T088 manual validation tasks can run in parallel

---

## Parallel Example: User Story 1 (MVP)

```bash
# Launch all Pydantic models together:
Task T028: "Create backend/src/models/request.py with QueryRequest"
Task T029: "Create backend/src/models/response.py with CitationSource and QueryResponse"
Task T030: "Create backend/src/models/retrieval.py with RetrievedChunk"

# Later, launch all grounding service methods together:
Task T041: "Implement is_out_of_scope in backend/src/services/grounding_service.py"
Task T042: "Implement generate_refusal_message in backend/src/services/grounding_service.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 + 6 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T027) - **CRITICAL BLOCKING PHASE**
3. Complete Phase 3: User Story 1 + 6 (T028-T046) - **MVP COMPLETE**
4. **STOP and VALIDATE**: Test backend API with curl/Postman
   - POST /query with {"q": "What is ROS 2?"} → verify answer with citations
   - POST /query with {"q": "What is the capital of France?"} → verify refusal message (zero hallucinations)
5. Skip to Phase 8: Minimal deployment (T072-T079) to test in production
6. Deploy MVP and demo

**At this point you have a working RAG chatbot backend with zero hallucinations!**

### Incremental Delivery

1. **Foundation** (Phases 1-2) → Backend infrastructure ready
2. **MVP** (Phase 3) → US1+US6 complete: Ask questions with zero hallucinations ✅ **DELIVERABLE 1**
3. **+ Logging** (Phase 4) → US3 complete: All interactions logged ✅ **DELIVERABLE 2**
4. **+ Citations** (Phase 5) → US4 complete: Answers link to chapters ✅ **DELIVERABLE 3**
5. **+ Selection Mode** (Phase 6) → US2 complete: Highlighted text bypass ✅ **DELIVERABLE 4**
6. **+ Widget** (Phase 7) → US5 complete: Full UI in Docusaurus ✅ **DELIVERABLE 5**
7. **Production** (Phase 8) → Deployed and live ✅ **DELIVERABLE 6**
8. **Polish** (Phase 9) → Production-ready ✅ **DELIVERABLE 7**

Each increment adds value without breaking previous features.

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (Phases 1-2)
2. Once Foundational is done:
   - **Developer A**: User Story 1+6 (Phase 3 - T028-T046) - Backend RAG core
   - **Developer B**: Populate book_index (Phase 5 - T049-T050) + Frontend foundation (Phase 7 - T057-T060)
   - **Developer C**: Deployment setup (Phase 8 - T072-T074) + Documentation (Phase 9 - T080-T082)
3. **Checkpoint**: US1+6 complete → Integrate US3, US4, US2
4. **Developer A**: US3 Logging (Phase 4) → US4 Citations (Phase 5 - T051-T053)
5. **Developer B**: US5 Widget integration (Phase 7 - T061-T071)
6. **Developer C**: US2 Selection mode (Phase 6)
7. **All**: Deploy, validate, polish

---

## Notes

- **[P] tasks** = different files, no dependencies - can run in parallel
- **[Story] label** maps task to specific user story (US1, US2, US3, US4, US5, US6) for traceability
- **User Story 1 + 6 (Phase 3) is the MVP** - delivers core chatbot functionality with zero hallucinations
- **User Story 4 (citations) depends on US1** - cannot generate citations without retrieval working
- **User Story 5 (widget) depends on US1** - needs /query endpoint to exist
- **User Story 2 (selection mode) depends on US1** - reuses OpenAI service
- **User Story 3 (logging) is independent** - can be added anytime after US1 works
- Each user story should be independently testable at its checkpoint
- Commit after each task or logical group
- Validate tests/checkpoints before proceeding to next phase
- **Zero hallucinations (US6) is enforced throughout US1 implementation** - not a separate phase
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Count Summary

- **Phase 1 (Setup)**: 7 tasks
- **Phase 2 (Foundational)**: 20 tasks (T008-T027)
- **Phase 3 (US1+US6 - MVP)**: 19 tasks (T028-T046)
- **Phase 4 (US3)**: 2 tasks (T047-T048)
- **Phase 5 (US4)**: 5 tasks (T049-T053)
- **Phase 6 (US2)**: 3 tasks (T054-T056)
- **Phase 7 (US5)**: 15 tasks (T057-T071)
- **Phase 8 (Deployment)**: 8 tasks (T072-T079)
- **Phase 9 (Polish)**: 11 tasks (T080-T090)

**Total**: 90 tasks

**MVP Scope** (Minimum viable product): Phases 1-3 = 46 tasks

**Parallel Opportunities**: 23 tasks marked [P] can be executed in parallel within their phases

**Independent Test Criteria**:
- **US1+US6**: Backend API returns accurate, grounded answers with zero hallucinations
- **US2**: Highlighted text bypasses Qdrant and uses selection_text only
- **US3**: Chat logs persisted to Neon database
- **US4**: Citations link to correct book chapters
- **US5**: Widget embedded in Docusaurus, functional on all pages

**Suggested MVP**: Complete Phases 1-3 (T001-T046) for a working backend RAG chatbot API with zero hallucinations. This is independently deployable and testable via API calls.
