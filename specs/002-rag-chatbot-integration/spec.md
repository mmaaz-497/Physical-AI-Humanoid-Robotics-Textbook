j# Feature Specification: Integrated RAG Chatbot for Physical AI Book

**Feature Branch**: `002-rag-chatbot-integration`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Create a complete specification for the 'Integrated RAG Chatbot Development' feature of my Physical AI & Humanoid Robotics book project."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Book Content (Priority: P1)

Students reading the Physical AI & Humanoid Robotics textbook need to ask questions and receive accurate answers grounded exclusively in the book's content without leaving the reading interface.

**Why this priority**: This is the core value proposition - enabling readers to get immediate, contextual help while studying. Without this, the chatbot has no purpose. This is the MVP.

**Independent Test**: Can be fully tested by opening any chapter page, typing a question related to that chapter's content (e.g., "What is the difference between ROS 2 nodes and topics?"), and verifying that the chatbot returns an accurate answer with citations to specific book sections.

**Acceptance Scenarios**:

1. **Given** a student is reading "Module 1: ROS 2 Fundamentals", **When** they type "How do I create a ROS 2 node?", **Then** the chatbot retrieves relevant context from the book's vector database and generates an answer citing specific chapter sections
2. **Given** a student asks a question about content in the book, **When** the chatbot responds, **Then** the answer is grounded ONLY on retrieved book context and includes no external knowledge or hallucinations
3. **Given** a student asks a question unrelated to the book, **When** the chatbot searches the vector database, **Then** it responds with "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content. Your question appears to be outside the book's scope."

---

### User Story 2 - Get Context-Aware Answers from Highlighted Text (Priority: P2)

Students need to highlight specific text on the page and ask clarifying questions about that exact passage without the system searching the entire book.

**Why this priority**: This enhances precision by bypassing vector search when the user already knows the relevant context. It builds on P1 by adding a faster, more targeted interaction mode.

**Independent Test**: Can be fully tested by highlighting a code block or technical paragraph in any chapter, clicking the chatbot icon, asking "Explain this code", and verifying that the chatbot uses ONLY the highlighted text as context (no Qdrant retrieval).

**Acceptance Scenarios**:

1. **Given** a student highlights a URDF code snippet, **When** they ask "What does this code do?", **Then** the chatbot uses the highlighted text as context and bypasses Qdrant retrieval
2. **Given** a student highlights a definition, **When** they ask "Can you simplify this?", **Then** the chatbot provides a simplified explanation based solely on the highlighted text
3. **Given** a student submits a query with highlighted text, **When** the backend receives the request, **Then** the `selection_text` parameter is populated and Qdrant retrieval is skipped

---

### User Story 3 - Review Past Interactions (Priority: P3)

Students and educators need to review the history of questions asked and answers provided to track learning progress and identify knowledge gaps.

**Why this priority**: Logging enables analytics and debugging but is not required for the core chatbot functionality. This can be delivered after P1 and P2 are stable.

**Independent Test**: Can be fully tested by asking 5 different questions, then querying the Neon database `chat_logs` table and verifying that all 5 questions, answers, timestamps, and metadata are recorded correctly.

**Acceptance Scenarios**:

1. **Given** a student asks a question, **When** the chatbot responds, **Then** the question, answer, timestamp, and retrieval metadata are logged to the Neon `chat_logs` table
2. **Given** an educator reviews chat logs, **When** they query the database, **Then** they can identify frequently asked questions and topics requiring clarification
3. **Given** a developer debugs a poor answer, **When** they inspect the chat log entry, **Then** they can see which Qdrant chunks were retrieved and the prompt sent to the LLM

---

### User Story 4 - Navigate to Specific Chapters from Chatbot Answers (Priority: P4)

Students need clickable links in chatbot responses that navigate directly to the relevant book chapter or section where the answer was sourced.

**Why this priority**: Deep linking improves user experience by allowing students to verify answers and read full context. This depends on P1 working first and requires integration with the Docusaurus navigation system.

**Independent Test**: Can be fully tested by asking "What is Physical AI?", receiving an answer, and verifying that the response includes a clickable link (e.g., `/docs/introduction/what-is-physical-ai#definition`) that navigates to the exact section in the book.

**Acceptance Scenarios**:

1. **Given** the chatbot retrieves context from a specific chapter, **When** it generates an answer, **Then** the response includes a citation with a clickable link to the source chapter
2. **Given** a student clicks a citation link, **When** the page loads, **Then** the browser scrolls to the exact section referenced in the answer
3. **Given** the `book_index` table maps Qdrant IDs to file paths, **When** the chatbot retrieves a chunk, **Then** it looks up the corresponding chapter URL and includes it in the citation

---

### User Story 5 - Embed Chatbot Widget in Web Version (Priority: P5)

The chatbot must be accessible as a floating widget on every page of the Docusaurus-based Physical AI textbook website, allowing students to invoke it from any chapter.

**Why this priority**: This enables universal access across the book. It depends on the backend API (P1) and requires frontend integration with Docusaurus.

**Independent Test**: Can be fully tested by opening any chapter page in the Docusaurus site, clicking the chatbot widget (floating icon in bottom-right corner), typing a question, and verifying the widget displays the response inline.

**Acceptance Scenarios**:

1. **Given** a student visits any chapter page, **When** the page loads, **Then** a chatbot widget icon appears in the bottom-right corner
2. **Given** a student clicks the chatbot icon, **When** the widget opens, **Then** they see a text input field, a "Send" button, and a conversation history pane
3. **Given** a student types a question and clicks "Send", **When** the request is sent to the backend `/query` endpoint, **Then** the response is displayed in the widget conversation pane within 5 seconds

---

### User Story 6 - Limit Responses to Book Content Only (Priority: P1 - Critical Constraint)

The chatbot must NEVER hallucinate or provide information outside the book's content, even if the LLM has broader knowledge.

**Why this priority**: This is a critical constraint that ensures trust and accuracy. It's part of P1 because it must be enforced from day one.

**Independent Test**: Can be fully tested by asking intentionally out-of-scope questions (e.g., "What is the capital of France?") and verifying the chatbot refuses to answer or responds with "This question is outside the scope of the Physical AI textbook."

**Acceptance Scenarios**:

1. **Given** a student asks a question unrelated to the book, **When** Qdrant returns no relevant chunks (similarity score below threshold), **Then** the chatbot responds with a polite refusal message
2. **Given** the LLM generates a response, **When** the system validates the answer, **Then** it ensures the answer is grounded in the retrieved context and does not introduce external facts
3. **Given** a question is ambiguous, **When** no clear answer exists in the book, **Then** the chatbot responds with "I couldn't find information on this topic in the textbook. Please try rephrasing or ask about a different topic."

---

### Edge Cases

- What happens when Qdrant Cloud is unreachable or times out during retrieval?
- How does the system handle questions in languages other than English?
- What happens when a student submits an empty query or very long query (>1000 characters)?
- How does the chatbot respond when the book index is out of sync with Qdrant embeddings (e.g., chapter renamed)?
- What happens when the OpenAI API rate limit is exceeded or the API key is invalid?
- How does the widget behave when JavaScript is disabled in the browser?
- What happens when the Neon database connection fails during logging?
- How does the system handle concurrent requests from multiple users?
- What happens if the user highlights text but the selection is lost before submission?
- How does the chatbot handle code blocks or special characters in user queries?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST retrieve context from Qdrant Cloud vector database using semantic similarity search with `text-embedding-3-small` (1536-dimensional embeddings)
- **FR-002**: System MUST use OpenAI GPT-4o-mini (or user-specified model) for answer generation
- **FR-003**: System MUST ground all answers ONLY on retrieved book context and MUST NOT hallucinate or use external knowledge
- **FR-004**: System MUST expose a FastAPI `/query` endpoint accepting parameters: `q` (question string), `top_k` (number of chunks to retrieve, default 5), `selection_text` (optional highlighted text)
- **FR-005**: System MUST bypass Qdrant retrieval when `selection_text` is provided and use the highlighted text as the sole context
- **FR-006**: System MUST log all queries and responses to Neon Serverless Postgres `chat_logs` table with fields: `id`, `question`, `answer`, `timestamp`, `retrieval_metadata`, `model_used`
- **FR-007**: System MUST maintain a `book_index` table in Neon Postgres mapping `qdrant_id` → `chapter_title` → `file_path` → `section_heading`
- **FR-008**: System MUST return responses in JSON format with fields: `answer` (string), `sources` (array of citation objects with `chapter`, `file_path`, `chunk_text`), `confidence` (float 0-1)
- **FR-009**: System MUST include citations in chatbot responses linking each answer segment to the source chapter/section
- **FR-010**: System MUST refuse to answer questions when no relevant context is found (similarity score below threshold, e.g., 0.6)
- **FR-011**: System MUST provide a universal JavaScript widget that can be embedded in any web page (Docusaurus or standalone)
- **FR-012**: Widget MUST support highlighting text on the page and passing it as `selection_text` parameter
- **FR-013**: Widget MUST display a conversation history with user questions and chatbot responses
- **FR-014**: Widget MUST handle loading states, error states, and retry logic for failed requests
- **FR-015**: System MUST enforce a maximum query length of 500 characters and return an error for longer queries
- **FR-016**: System MUST cap `top_k` retrieval to a maximum of 10 chunks to prevent excessive token usage
- **FR-017**: System MUST use environment variables for all secrets: `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION`, `OPENAI_API_KEY`, `NEON_DATABASE_URL`
- **FR-018**: System MUST validate Qdrant connection and OpenAI API key at startup and fail fast if credentials are invalid
- **FR-019**: System MUST return user-friendly error messages for common failure modes (API down, rate limit exceeded, invalid query)
- **FR-020**: System MUST include CORS headers to allow requests from the Docusaurus frontend domain

### Key Entities

- **Query**: User question submitted to the chatbot. Attributes: `question_text`, `selection_text` (optional), `top_k`, `timestamp`, `user_session_id` (optional)
- **RetrievedChunk**: A segment of book content retrieved from Qdrant. Attributes: `qdrant_id`, `chunk_text`, `similarity_score`, `metadata` (chapter, section, file_path)
- **Answer**: Generated response from the LLM. Attributes: `answer_text`, `sources` (array of RetrievedChunk references), `confidence_score`, `model_used`
- **ChatLog**: Database record of a query-answer interaction. Attributes: `id`, `question`, `answer`, `timestamp`, `retrieval_metadata` (JSON), `model_used`, `session_id`
- **BookIndex**: Mapping between Qdrant vector IDs and book structure. Attributes: `qdrant_id`, `chapter_title`, `file_path`, `section_heading`, `page_number` (optional)
- **ChatbotWidget**: Frontend JavaScript component. Attributes: `open_state` (boolean), `conversation_history` (array of message objects), `loading_state`, `error_state`

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot returns accurate, grounded answers for 95% of in-scope questions (measured by human evaluation of 100 test questions)
- **SC-002**: Chatbot correctly refuses to answer out-of-scope questions 100% of the time (measured by testing with 20 intentionally out-of-scope questions)
- **SC-003**: Average response latency is under 5 seconds from query submission to answer display (measured at p95)
- **SC-004**: Chatbot widget loads on all Docusaurus pages within 2 seconds of page load
- **SC-005**: 100% of chat interactions are successfully logged to Neon database (verified by comparing request count to log count)
- **SC-006**: Citation links navigate to the correct chapter/section 100% of the time (verified by testing 50 random answers)
- **SC-007**: Highlighted text bypass mode works in 100% of cases where `selection_text` is provided (verified by testing 20 queries with highlighted text)
- **SC-008**: System handles at least 100 concurrent users without degradation (verified by load testing)
- **SC-009**: Widget displays user-friendly error messages for all failure modes (API down, rate limit, network error)
- **SC-010**: All secrets are managed via environment variables with zero hardcoded credentials in source code

### Assumptions

- **A-001**: Book content is already embedded and uploaded to Qdrant Cloud Free Tier (1536-dim vectors from `text-embedding-3-small`)
- **A-002**: Qdrant collection name is known and will be provided via `QDRANT_COLLECTION` environment variable
- **A-003**: Neon Serverless Postgres Free Tier has sufficient capacity for chat logs and book index (< 10,000 queries/month expected)
- **A-004**: OpenAI API usage will stay within rate limits (gpt-4o-mini has higher limits than gpt-4)
- **A-005**: The Docusaurus site allows custom JavaScript injection for the chatbot widget
- **A-006**: Users primarily interact with the chatbot in English (multilingual support is out of scope)
- **A-007**: The book index table is manually created and kept in sync with Qdrant embeddings (automated sync is out of scope for MVP)
- **A-008**: The chatbot will be used primarily for educational purposes, not adversarial testing or abuse
- **A-009**: Hosting environment supports FastAPI deployment (e.g., Render, Railway, Fly.io, or AWS Lambda)
- **A-010**: The widget will be tested on modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)

### Non-Functional Requirements

- **NFR-001**: Backend API MUST respond to `/query` requests within 5 seconds at p95 latency
- **NFR-002**: System MUST handle up to 100 concurrent requests without errors
- **NFR-003**: Chatbot widget MUST be responsive and functional on desktop, tablet, and mobile viewports
- **NFR-004**: Widget MUST be accessible via keyboard navigation (Tab to input, Enter to submit, Esc to close)
- **NFR-005**: System MUST log all errors to stdout/stderr for debugging in production
- **NFR-006**: API MUST use HTTPS for all communication between frontend and backend
- **NFR-007**: Database credentials and API keys MUST be stored in environment variables (never in code or config files)
- **NFR-008**: System MUST gracefully degrade when Qdrant or OpenAI APIs are unavailable (return user-friendly error, not crash)
- **NFR-009**: Widget JavaScript bundle MUST be under 50KB gzipped for fast page loads
- **NFR-010**: All code MUST be documented with docstrings and inline comments for maintainability

## Scope

### In Scope

- FastAPI backend with `/query` endpoint
- Qdrant Cloud integration for vector retrieval
- OpenAI API integration for answer generation (gpt-4o-mini)
- Neon Serverless Postgres for chat logs and book index
- Answer grounding: all responses must cite retrieved context
- Highlighted text bypass mode (skip Qdrant when `selection_text` is provided)
- Universal JavaScript chatbot widget for web embedding
- Widget integration with Docusaurus book site
- Environment variable management for secrets
- Error handling and user-friendly error messages
- Citation generation with chapter/section links
- Conversation history display in widget
- Query validation (length limits, parameter validation)
- CORS configuration for cross-origin requests
- Local development setup instructions
- Production deployment outline (hosting platform TBD)

### Out of Scope

- User authentication or session management (chatbot is anonymous)
- Multi-language support (English only)
- Voice input or text-to-speech output
- Admin dashboard for reviewing chat logs
- Automated retraining or updating of Qdrant embeddings
- Integration with Learning Management Systems (LMS)
- Chatbot analytics dashboard (beyond raw database logs)
- Rate limiting per user (rely on hosting platform or OpenAI rate limits)
- Custom embedding models (locked to `text-embedding-3-small`)
- Streaming responses (full response returned at once)
- Multi-turn conversation context (each query is stateless)
- Feedback mechanism for rating answer quality
- PDF or eBook version of the chatbot
- Mobile app version of the chatbot
- Integration with other AI assistants (e.g., Siri, Alexa)

## Dependencies

### External Dependencies

- **Qdrant Cloud Free Tier**: Vector database hosting book embeddings (1536-dim vectors)
- **OpenAI API**: GPT-4o-mini model for answer generation and `text-embedding-3-small` for query embeddings
- **Neon Serverless Postgres**: Database for chat logs and book index
- **FastAPI**: Python web framework for backend API
- **Python 3.10+**: Runtime for FastAPI backend
- **JavaScript (ES6+)**: Runtime for chatbot widget
- **Docusaurus**: Static site generator hosting the book (widget must integrate with Docusaurus pages)
- **Hosting Platform**: Backend deployment (e.g., Render, Railway, Fly.io, AWS Lambda)
- **HTTPS Certificate**: Required for secure communication between frontend and backend

### Internal Dependencies

- **Book Content**: Already embedded and uploaded to Qdrant Cloud (prerequisite)
- **Book Index Mapping**: `book_index` table must be populated with Qdrant ID → chapter → file path mappings before chatbot can provide citations
- **Docusaurus Site**: Feature 001 (Docusaurus Book Structure) must be deployed before widget can be embedded
- **Environment Variables**: `.env` file or hosting platform environment variable configuration for secrets

## Constraints

- **C-001**: All answers MUST be grounded exclusively on retrieved book content (no external knowledge or hallucinations)
- **C-002**: System MUST use Qdrant Cloud Free Tier (budget constraint: no paid vector database)
- **C-003**: System MUST use Neon Serverless Postgres Free Tier (budget constraint: no paid database)
- **C-004**: System MUST use OpenAI gpt-4o-mini (budget constraint: cost-effective model)
- **C-005**: Chatbot widget MUST be embeddable in any web page (not Docusaurus-specific)
- **C-006**: No hardcoded secrets or API keys in source code (all via environment variables)
- **C-007**: Query length MUST be capped at 500 characters to prevent abuse
- **C-008**: `top_k` retrieval MUST be capped at 10 chunks to prevent excessive token usage
- **C-009**: System MUST fail gracefully when external APIs are unavailable (no crashes)
- **C-010**: All database migrations MUST be reversible (up/down migrations)

## Risks

### Technical Risks

- **R-001**: Qdrant Cloud Free Tier rate limits or quota exceeded under high traffic
  - *Mitigation*: Monitor usage; implement client-side rate limiting; upgrade to paid tier if needed

- **R-002**: OpenAI API rate limits exceeded or costs exceed budget
  - *Mitigation*: Use gpt-4o-mini (cheaper than gpt-4); monitor costs; implement caching for common queries

- **R-003**: Neon Serverless Postgres free tier storage limit exceeded
  - *Mitigation*: Set retention policy for chat logs (e.g., auto-delete logs older than 90 days); monitor database size

- **R-004**: Book index table falls out of sync with Qdrant embeddings (chapters renamed or restructured)
  - *Mitigation*: Document manual sync process; implement automated validation script to detect mismatches

- **R-005**: Chatbot widget conflicts with Docusaurus JavaScript or CSS
  - *Mitigation*: Use shadow DOM for widget isolation; namespace all CSS classes; test on Docusaurus demo site

### Content Risks

- **R-006**: LLM generates hallucinated answers despite grounding instructions
  - *Mitigation*: Use strict prompt engineering ("Answer ONLY using the provided context"); implement post-generation validation; log all answers for manual review

- **R-007**: Retrieved chunks are too short or too long, resulting in poor context quality
  - *Mitigation*: Test different chunk sizes (256, 512, 1024 tokens); adjust during Qdrant data preparation if needed

- **R-008**: User queries are too vague or ambiguous, leading to irrelevant answers
  - *Mitigation*: Return "I need more context" response when similarity scores are low; suggest example questions

### User Experience Risks

- **R-009**: Chatbot response latency exceeds 5 seconds, frustrating users
  - *Mitigation*: Optimize Qdrant query; use faster OpenAI model (gpt-4o-mini); implement loading indicators in widget

- **R-010**: Citations are not clear or clickable links don't work
  - *Mitigation*: Test citation formatting; ensure `book_index` URLs are valid; validate links before deployment

- **R-011**: Widget is not mobile-friendly or inaccessible to screen readers
  - *Mitigation*: Test on mobile devices; implement ARIA labels; ensure keyboard navigation works

### Operational Risks

- **R-012**: Secrets leaked in source code or logs
  - *Mitigation*: Use `.env` file for local development (add to `.gitignore`); use platform environment variables in production; scan commits for secrets

- **R-013**: Backend API becomes unavailable, breaking chatbot on all pages
  - *Mitigation*: Deploy backend to reliable hosting platform; implement health check endpoint; set up uptime monitoring

## Open Questions

**Q-001**: Which hosting platform should be used for the FastAPI backend?
*Options*: Render (free tier available), Railway (usage-based pricing), Fly.io (global edge deployment), AWS Lambda (serverless)
*Decision needed*: User should specify preference based on budget and performance requirements.

**Q-002**: Should the widget support multi-turn conversations (remembering previous questions in the session)?
*Current assumption*: No, each query is stateless (simpler implementation).
*Tradeoff*: Multi-turn conversations improve UX but require session management and more complex prompting.

**Q-003**: Should the system implement caching for common queries to reduce API costs?
*Current assumption*: No caching in MVP (simpler implementation).
*Tradeoff*: Caching reduces costs and latency but adds complexity and requires cache invalidation strategy.

**Q-004**: Should the book index table be manually maintained or auto-generated from Qdrant metadata?
*Current assumption*: Manually maintained in MVP.
*Tradeoff*: Manual maintenance is error-prone but simpler; auto-generation requires additional tooling.

**Q-005**: Should the widget allow users to rate answer quality (thumbs up/down)?
*Current assumption*: Not in MVP (out of scope).
*Tradeoff*: User feedback improves quality over time but requires additional UI and database schema.

**Q-006**: What is the retention policy for chat logs in the Neon database?
*Current assumption*: Indefinite retention until free tier storage limit is reached.
*Decision needed*: Specify retention period (e.g., 90 days, 1 year) or implement auto-deletion policy.

**Q-007**: Should the system support custom prompts or system messages configurable via environment variables?
*Current assumption*: No, use hardcoded prompt optimized for grounding.
*Tradeoff*: Configurable prompts allow experimentation but increase complexity and risk of misconfiguration.

**Q-008**: Should the widget support exporting conversation history to text or JSON?
*Current assumption*: Not in MVP.
*Tradeoff*: Export feature useful for students saving Q&A for notes but adds UI complexity.
