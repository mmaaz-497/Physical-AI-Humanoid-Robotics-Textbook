# Implementation Tasks: Better Auth Authentication System

**Feature**: Better Auth Authentication System
**Branch**: `001-better-auth`
**Generated**: 2025-12-14

## Overview

This task list implements authentication for the Docusaurus documentation platform using Better Auth (TypeScript/Node.js framework) alongside the existing FastAPI backend.

**Architecture**: Microservices
- Auth Service: Node.js/TypeScript + Better Auth + Drizzle ORM
- Backend API: FastAPI + JWT validation
- Frontend: Docusaurus/React + @better-auth/react
- Database: Shared PostgreSQL (Neon)

**User Stories** (from spec.md):
1. **US1** (P1): New User Signup with Background Collection
2. **US2** (P1): Existing User Signin
3. **US3** (P2): User Logout
4. **US4** (P2): Session Persistence and Management

## Implementation Strategy

**MVP Scope** (Minimum Viable Product):
- User Story 1 (Signup) + User Story 2 (Signin) = Functional authentication

**Incremental Delivery**:
1. Phase 1-2: Setup + Foundation → enables all user stories
2. Phase 3: US1 (Signup) → first testable increment
3. Phase 4: US2 (Signin) → completes MVP
4. Phase 5: US3 (Logout) → enhances security
5. Phase 6: US4 (Session Management) → production-ready
6. Phase 7: Polish → deployment-ready

**Parallel Execution**: Tasks marked with `[P]` can be executed in parallel within their phase (different files, no dependencies).

---

## Phase 1: Project Setup & Infrastructure

**Goal**: Initialize auth service project and configure shared infrastructure

**Tasks**:

- [ ] T001 Create auth-service directory and initialize Node.js project with TypeScript in auth-service/
- [ ] T002 [P] Install Better Auth dependencies (better-auth, @better-auth/drizzle, hono) in auth-service/package.json
- [ ] T003 [P] Install Drizzle ORM dependencies (drizzle-orm, drizzle-kit, postgres) in auth-service/package.json
- [ ] T004 [P] Install development dependencies (typescript, vitest, @types/node, tsx) in auth-service/package.json
- [ ] T005 Create TypeScript configuration in auth-service/tsconfig.json with strict mode and ESNext target
- [ ] T006 [P] Create Drizzle configuration in auth-service/drizzle.config.ts pointing to PostgreSQL Neon database
- [ ] T007 [P] Create environment variables template in auth-service/.env.example with DATABASE_URL, JWT_SECRET, PORT, CORS_ORIGINS
- [ ] T008 Set up actual environment variables in auth-service/.env (copy from .env.example and fill with real values)
- [ ] T009 [P] Update FastAPI requirements.txt to add python-jose[cryptography]==3.3.0 for JWT validation
- [ ] T010 [P] Install Frontend Better Auth React client (@better-auth/react) in root package.json
- [ ] T011 Create Docker Compose configuration in docker-compose.yml for auth-service, backend, and PostgreSQL
- [ ] T012 Add npm scripts to auth-service/package.json for dev, build, test, migrate, and generate

**Acceptance Criteria**:
- ✅ Auth service project initialized with TypeScript
- ✅ All dependencies installed (auth service, backend, frontend)
- ✅ Environment variables configured
- ✅ Docker Compose ready for local development
- ✅ Can run `npm run dev` in auth-service (even if nothing works yet)

---

## Phase 2: Foundational Services (Blocking Prerequisites)

**Goal**: Set up core database schema and authentication configuration that all user stories depend on

**Tasks**:

- [ ] T013 Define Drizzle database schema in auth-service/src/db/schema.ts with user, session, account, verification tables
- [ ] T014 Add custom user fields to schema (experienceLevel, professionalRole, roleOther, organization) in auth-service/src/db/schema.ts
- [ ] T015 Create database client in auth-service/src/db/client.ts using drizzle-orm with PostgreSQL connection
- [ ] T016 Generate and run Drizzle migrations to create Better Auth tables in shared PostgreSQL database
- [ ] T017 Configure Better Auth instance in auth-service/src/lib/auth.ts with email/password, custom fields, and session settings
- [ ] T018 Create Hono app entry point in auth-service/src/index.ts with CORS middleware and Better Auth routes
- [ ] T019 [P] Configure CORS middleware in auth-service/src/middleware/cors.ts to allow Docusaurus frontend origin
- [ ] T020 [P] Create JWT validation middleware in backend/src/middleware/auth.py using python-jose to verify Better Auth tokens
- [ ] T021 [P] Create JWT payload Pydantic schemas in backend/src/models/auth_schemas.py for type safety
- [ ] T022 Update FastAPI config in backend/src/config.py to add BETTER_AUTH_JWT_SECRET environment variable
- [ ] T023 Create Better Auth React client in src/lib/auth-client.ts configured with auth service base URL

**Acceptance Criteria**:
- ✅ Database tables created in PostgreSQL (user, session, account, verification)
- ✅ Better Auth configured with custom user profile fields
- ✅ Auth service server starts and responds to health checks
- ✅ CORS configured for Docusaurus frontend
- ✅ FastAPI can validate JWT tokens (integration ready)
- ✅ Frontend has Better Auth client configured

**Independent Test**: Start auth service and verify `/api/auth/session` endpoint returns 401 (unauthenticated) - proves service is running and auth is enforced.

---

## Phase 3: User Story 1 - New User Signup with Background Collection (P1)

**Story Goal**: A new visitor can create an account with email/password and provide background information (experience level, professional role) for future personalization.

**Independent Test**: Fill out signup form with valid credentials and background questions → Submit → Verify account created in database with all fields stored → User automatically logged in → Redirected to homepage.

**Tasks**:

### Backend (Auth Service)

- [ ] T024 [US1] Verify Better Auth email/password signup endpoint is enabled at POST /api/auth/signup/email in auth-service/src/lib/auth.ts
- [ ] T025 [US1] Configure password validation rules (min 8 chars) in Better Auth config in auth-service/src/lib/auth.ts
- [ ] T026 [US1] Add custom field validation for experienceLevel and professionalRole in auth-service/src/lib/auth.ts
- [ ] T027 [US1] Test signup endpoint with curl/Postman to verify account creation with custom fields

### Frontend (Docusaurus)

- [ ] T028 [P] [US1] Create signup page component in src/pages/auth/signup.tsx with form fields for email, password, name
- [ ] T029 [US1] Add experience level dropdown to signup form with options (beginner, intermediate, advanced) in src/pages/auth/signup.tsx
- [ ] T030 [US1] Add professional role dropdown with options (student, researcher, engineer, hobbyist, other) in src/pages/auth/signup.tsx
- [ ] T031 [US1] Add conditional "other" text field that appears when role=other in src/pages/auth/signup.tsx
- [ ] T032 [US1] Add optional organization text field to signup form in src/pages/auth/signup.tsx
- [ ] T033 [US1] Implement form validation with real-time feedback for email format and password strength in src/pages/auth/signup.tsx
- [ ] T034 [US1] Connect signup form to Better Auth signUp.email() method in src/pages/auth/signup.tsx
- [ ] T035 [US1] Handle signup errors (duplicate email, validation failures) with user-friendly messages in src/pages/auth/signup.tsx
- [ ] T036 [US1] Redirect to homepage after successful signup in src/pages/auth/signup.tsx
- [ ] T037 [P] [US1] Add "Sign Up" button to Docusaurus navbar using swizzled Navbar component in src/theme/Navbar/index.tsx
- [ ] T038 [P] [US1] Style signup page to match Docusaurus theme with mobile-responsive layout in src/css/auth.css

### Integration & Testing

- [ ] T039 [US1] Create Auth Context Provider in src/contexts/AuthContext.tsx using useSession hook from Better Auth React
- [ ] T040 [US1] Wrap Docusaurus app with AuthProvider in src/theme/Root.js
- [ ] T041 [US1] Test complete signup flow: homepage → click signup → fill form → submit → verify DB → verify logged in state

**Acceptance Criteria**:
- ✅ Visitor can navigate from homepage to signup page
- ✅ Form validates email format and password strength in real-time
- ✅ Submitting valid data creates account in database with all custom fields
- ✅ User automatically logged in after signup (session created)
- ✅ Duplicate email shows clear error message
- ✅ Mobile-responsive (works on 320px width screens)

---

## Phase 4: User Story 2 - Existing User Signin (P1)

**Story Goal**: A returning user can sign in with email/password to access their account and personalized content.

**Independent Test**: Create test user via signup → Logout → Navigate to signin page → Enter correct credentials → Submit → Verify logged in → Session persists across page navigation.

**Tasks**:

### Backend (Auth Service)

- [ ] T042 [US2] Verify Better Auth email/password signin endpoint is enabled at POST /api/auth/signin/email in auth-service/src/lib/auth.ts
- [ ] T043 [US2] Test signin endpoint with curl/Postman using test account credentials

### Frontend (Docusaurus)

- [ ] T044 [P] [US2] Create signin page component in src/pages/auth/login.tsx with email and password fields
- [ ] T045 [US2] Connect signin form to Better Auth signIn.email() method in src/pages/auth/login.tsx
- [ ] T046 [US2] Handle signin errors (invalid credentials, network errors) with generic security message in src/pages/auth/login.tsx
- [ ] T047 [US2] Redirect to homepage after successful signin in src/pages/auth/login.tsx
- [ ] T048 [US2] Implement session persistence check on app mount using useSession hook in src/contexts/AuthContext.tsx
- [ ] T049 [US2] Display user email/name in navbar when authenticated in src/theme/Navbar/index.tsx
- [ ] T050 [P] [US2] Add "Sign In" button to Docusaurus navbar in src/theme/Navbar/index.tsx
- [ ] T051 [P] [US2] Style signin page to match Docusaurus theme with mobile-responsive layout in src/css/auth.css

### Integration & Testing

- [ ] T052 [US2] Test complete signin flow: homepage → click signin → enter credentials → submit → verify logged in
- [ ] T053 [US2] Test session persistence: signin → navigate to different pages → refresh browser → verify still logged in
- [ ] T054 [US2] Test invalid credentials show error message without revealing which field is wrong

**Acceptance Criteria**:
- ✅ User can navigate from homepage to signin page
- ✅ Valid credentials log user in and redirect to homepage
- ✅ Invalid credentials show generic error (no user enumeration)
- ✅ Session persists across page navigation
- ✅ Session persists after browser refresh (if not expired)
- ✅ User email/name displayed in navbar when logged in
- ✅ Mobile-responsive signin page

---

## Phase 5: User Story 3 - User Logout (P2)

**Story Goal**: An authenticated user can securely log out, terminating their session and preventing access to protected features.

**Independent Test**: Sign in as user → Verify logged in (see email in navbar) → Click logout → Verify redirected to homepage → Verify session destroyed → Attempt to access protected feature → Verify prompted to sign in.

**Tasks**:

### Backend (Auth Service)

- [ ] T055 [US3] Verify Better Auth signout endpoint is enabled at POST /api/auth/signout in auth-service/src/lib/auth.ts
- [ ] T056 [US3] Verify signout revokes session in database and clears cookies

### Frontend (Docusaurus)

- [ ] T057 [US3] Add "Logout" button to navbar (only visible when authenticated) in src/theme/Navbar/index.tsx
- [ ] T058 [US3] Connect logout button to Better Auth signOut() method in src/theme/Navbar/index.tsx
- [ ] T059 [US3] Redirect to homepage after logout in src/theme/Navbar/index.tsx
- [ ] T060 [US3] Update navbar to show "Sign In" button after logout (remove user email) in src/theme/Navbar/index.tsx

### Integration & Testing

- [ ] T061 [US3] Test complete logout flow: signin → verify logged in → click logout → verify logged out → verify navbar updated
- [ ] T062 [US3] Test session destruction: logout → try accessing protected feature → verify redirected to signin
- [ ] T063 [US3] Test browser back button after logout doesn't allow access to protected content

**Acceptance Criteria**:
- ✅ Logout button visible only when user is logged in
- ✅ Clicking logout terminates session and clears cookies
- ✅ User redirected to homepage after logout
- ✅ Navbar updates to show "Sign In" button (removes user info)
- ✅ Cannot access protected features after logout without re-authenticating
- ✅ Browser back button doesn't bypass logout

---

## Phase 6: User Story 4 - Session Persistence and Management (P2)

**Story Goal**: The system automatically manages sessions with proper expiration, renewal, and multi-device support while maintaining security.

**Independent Test**: Sign in → Navigate across pages → Verify session persists → Wait for session expiration → Verify prompted to re-authenticate → Sign in on second device → Verify both sessions work independently.

**Tasks**:

### Backend (Auth Service)

- [ ] T064 [US4] Configure session expiration time (7 days) in Better Auth config in auth-service/src/lib/auth.ts
- [ ] T065 [US4] Configure session renewal on activity (update every 24 hours) in auth-service/src/lib/auth.ts
- [ ] T066 [US4] Verify Better Auth tracks IP address and user agent in session table for security monitoring
- [ ] T067 [US4] Test session refresh endpoint POST /api/auth/session/refresh with curl/Postman

### Frontend (Docusaurus)

- [ ] T068 [US4] Implement automatic session refresh in Auth Context when approaching expiration in src/contexts/AuthContext.tsx
- [ ] T069 [US4] Handle session expiration gracefully (redirect to login with message) in src/contexts/AuthContext.tsx
- [ ] T070 [US4] Test session persistence across page navigation using useSession hook in src/contexts/AuthContext.tsx

### Integration & Testing

- [ ] T071 [US4] Test session persists across page navigation without re-authentication
- [ ] T072 [US4] Test session renewal on user activity (navigate after 24 hours, verify session updated)
- [ ] T073 [US4] Test multi-device sessions: signin on device 1 → signin on device 2 → verify both sessions work independently
- [ ] T074 [US4] Test session expiration: wait for 7+ days → verify prompted to re-authenticate

**Acceptance Criteria**:
- ✅ Session expires after 7 days of inactivity
- ✅ Session renewed automatically on user activity (every 24 hours)
- ✅ User can sign in on multiple devices with independent sessions
- ✅ Expired sessions prompt re-authentication with clear message
- ✅ Session data includes IP address and user agent for security monitoring
- ✅ No unnecessary re-authentications during normal usage

---

## Phase 7: FastAPI Integration & Cross-Cutting Concerns

**Goal**: Integrate JWT validation in FastAPI backend and add polish for production deployment

**Tasks**:

### FastAPI JWT Validation

- [ ] T075 [P] Create get_current_user dependency in backend/src/middleware/auth.py that validates JWT and returns user data
- [ ] T076 [P] Add optional authentication support (allow unauthenticated but provide user if authenticated) in backend/src/middleware/auth.py
- [ ] T077 Update existing RAG query endpoint in backend/src/routers/query.py to accept optional user authentication
- [ ] T078 Log user activity (user_id, experience_level) when authenticated user queries RAG in backend/src/routers/query.py
- [ ] T079 Create FastAPI test for JWT validation in backend/tests/test_jwt_validation.py
- [ ] T080 Test protected endpoint requires valid JWT token in backend/tests/test_jwt_validation.py
- [ ] T081 Test expired JWT token returns 401 Unauthorized in backend/tests/test_jwt_validation.py

### UI/UX Polish

- [ ] T082 [P] Add loading states to signup/signin forms (disable submit while processing) in src/pages/auth/signup.tsx and src/pages/auth/login.tsx
- [ ] T083 [P] Add password visibility toggle (eye icon) to password fields in src/pages/auth/signup.tsx and src/pages/auth/login.tsx
- [ ] T084 [P] Implement "Remember me" functionality (extend session duration) in src/pages/auth/login.tsx
- [ ] T085 [P] Add link to signin from signup page ("Already have an account?") in src/pages/auth/signup.tsx
- [ ] T086 [P] Add link to signup from signin page ("Don't have an account?") in src/pages/auth/login.tsx
- [ ] T087 [P] Create ProtectedContent component wrapper in src/components/ProtectedContent.tsx for future feature gating
- [ ] T088 Test all authentication pages are mobile-responsive (320px to 1920px widths)
- [ ] T089 Test all forms provide real-time validation feedback within 500ms

### Security & Performance

- [ ] T090 [P] Add rate limiting plugin to Better Auth (5 requests/minute for auth endpoints) in auth-service/src/lib/auth.ts
- [ ] T091 [P] Enable Better Auth logging plugin for security event monitoring in auth-service/src/lib/auth.ts
- [ ] T092 Verify all passwords hashed with bcrypt (Better Auth default) - check database entries
- [ ] T093 Verify session cookies are httpOnly, secure, and sameSite configured in auth-service/src/lib/auth.ts
- [ ] T094 Test authentication response times are <500ms p95 using load testing tool
- [ ] T095 Test token validation in FastAPI is <50ms using pytest benchmarks

### Documentation & Deployment

- [ ] T096 [P] Update root README.md with authentication setup instructions and architecture diagram
- [ ] T097 [P] Create deployment guide in docs/DEPLOYMENT.md with Docker Compose and production configuration
- [ ] T098 [P] Document environment variables in .env.example files for both auth service and backend
- [ ] T099 Test Docker Compose brings up all services (auth, backend, database) with `docker-compose up`
- [ ] T100 Verify zero breaking changes to existing Docusaurus documentation content (run full site build)
- [ ] T101 Create production deployment checklist in docs/DEPLOYMENT.md (HTTPS, CORS, JWT secrets, etc.)

**Acceptance Criteria**:
- ✅ FastAPI validates Better Auth JWT tokens correctly
- ✅ Optional authentication works (public content accessible, but user tracked if logged in)
- ✅ All UI forms have loading states and validation feedback
- ✅ Authentication pages fully mobile-responsive
- ✅ Rate limiting prevents brute force attacks
- ✅ All security best practices implemented (httpOnly cookies, bcrypt, secure flags)
- ✅ Performance targets met (<500ms auth, <50ms token validation)
- ✅ Documentation complete for setup and deployment
- ✅ Docker Compose works for local development
- ✅ Zero breaking changes to existing site

---

## Dependencies & Execution Order

**Critical Path** (must complete in order):
1. Phase 1 (Setup) → Phase 2 (Foundation) → Phase 3 (US1 Signup)
2. Phase 3 (US1) → Phase 4 (US2 Signin) = **MVP Complete**
3. Phase 4 (US2) → Phase 5 (US3 Logout)
4. Phase 4 (US2) → Phase 6 (US4 Session Management)
5. Phase 2 (Foundation) → Phase 7 (FastAPI Integration)

**Parallel Opportunities Within Phases**:

**Phase 1** - All dependency installation tasks can run in parallel (T002, T003, T004, T006, T007, T009, T010)

**Phase 2** - Can run in parallel:
- Auth service DB setup (T013-T019)
- FastAPI JWT middleware (T020-T022)
- Frontend client setup (T023)

**Phase 3 (US1)** - Can run in parallel after T024-T027 complete:
- UI components (T028-T038) - multiple developers can work on different form fields
- T037 (navbar), T038 (styling), T039 (Auth Context) are independent

**Phase 4 (US2)** - Can run in parallel after T042-T043 complete:
- Signin page components (T044-T051)

**Phase 7** - Can run in parallel:
- FastAPI integration (T075-T081)
- UI polish (T082-T089)
- Security hardening (T090-T095)
- Documentation (T096-T101)

**Example Parallel Execution**:
```bash
# Phase 1 - Setup (4 developers in parallel)
Dev 1: T001-T005 (auth service TS setup)
Dev 2: T002, T003, T004 (install auth service deps)
Dev 3: T009 (FastAPI deps) + T010 (frontend deps)
Dev 4: T011-T012 (Docker + npm scripts)

# Phase 3 - US1 Signup (3 developers in parallel after backend ready)
Dev 1: T028-T033 (signup form fields)
Dev 2: T034-T036 (form submission & error handling)
Dev 3: T037, T038 (navbar + styling)
Then all: T039-T041 (integration & testing together)
```

---

## Testing Strategy

**Unit Tests** (if requested by user):
- Auth service: Test Better Auth configuration and custom field validation
- FastAPI: Test JWT validation logic
- Frontend: Test form validation logic

**Integration Tests**:
- Auth service + Database: Test signup/signin creates DB records
- FastAPI + Auth service: Test JWT token validation
- Frontend + Auth service: Test full authentication flows

**E2E Tests** (recommended):
- Complete signup flow (US1): Homepage → Signup → Fill form → Submit → Verify logged in
- Complete signin flow (US2): Logout → Signin → Verify logged in → Navigate pages
- Complete logout flow (US3): Signin → Logout → Verify logged out
- Session persistence (US4): Signin → Close browser → Reopen → Verify still logged in

**Performance Tests**:
- Auth response time <500ms p95 (load test signup/signin endpoints)
- Token validation <50ms (benchmark FastAPI JWT validation)
- Frontend form validation <500ms (measure real-time feedback delay)

---

## Task Summary

**Total Tasks**: 101

**By Phase**:
- Phase 1 (Setup): 12 tasks
- Phase 2 (Foundation): 11 tasks
- Phase 3 (US1 - Signup): 18 tasks
- Phase 4 (US2 - Signin): 13 tasks
- Phase 5 (US3 - Logout): 9 tasks
- Phase 6 (US4 - Session Management): 11 tasks
- Phase 7 (Polish & Integration): 27 tasks

**By User Story**:
- US1 (Signup): 18 tasks
- US2 (Signin): 13 tasks
- US3 (Logout): 9 tasks
- US4 (Session Management): 11 tasks
- Infrastructure: 23 tasks
- Cross-cutting: 27 tasks

**Parallel Opportunities**: 42 tasks marked with [P] can run in parallel within their phase

**MVP Scope** (minimum to demonstrate value):
- Phase 1, 2, 3 (US1), 4 (US2) = 54 tasks
- Delivers: Functional signup and signin with background data collection

**Estimated Effort**: 8-12 days
- Setup + Foundation: 2 days
- US1 + US2 (MVP): 4-5 days
- US3 + US4: 2 days
- Polish & Integration: 2-3 days

---

## Format Validation

✅ All tasks follow checklist format: `- [ ] [TaskID] [Optional:P] [Optional:Story] Description with file path`
✅ Task IDs sequential (T001-T101)
✅ Parallel tasks marked with [P]
✅ User story tasks marked with [US1], [US2], [US3], [US4]
✅ Each task includes specific file path
✅ Phases organized by user story priority (P1 stories before P2)
✅ Independent test criteria defined for each user story phase
✅ Dependencies and parallel execution documented
✅ MVP scope clearly identified

---

**Document Status**: ✅ COMPLETE - Ready for implementation

**Next Steps**:
1. Review tasks with team
2. Assign tasks to developers
3. Start with Phase 1 (Setup)
4. Complete MVP (Phases 1-4) first
5. Iterate on US3, US4, and polish
