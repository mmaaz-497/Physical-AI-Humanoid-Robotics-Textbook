# Authentication Testing Guide

This guide provides step-by-step instructions for testing the Better Auth authentication system.

## Prerequisites

Before testing, ensure you have:

1. **Database configured**: PostgreSQL connection string in `auth-service/.env`
2. **Services running**: Auth service (port 3001), Backend API (port 8000), Frontend (port 3000)
3. **Environment variables set**: All `.env` files configured correctly

## Starting the Services

### Option 1: Docker Compose (Recommended)

```bash
# Start all services
docker-compose up

# Or start in detached mode
docker-compose up -d

# View logs
docker-compose logs -f
```

### Option 2: Manual Start

```bash
# Terminal 1: Start auth service
cd auth-service
npm install
npm run dev

# Terminal 2: Start backend API
cd backend
pip install -r requirements.txt
uvicorn src.main:app --reload

# Terminal 3: Start frontend
npm install
npm start
```

## Test Scenarios

### Test 1: User Signup Flow

**Objective**: Verify new users can create accounts with background information.

**Steps**:

1. Navigate to `http://localhost:3000`
2. Click "Sign Up" button in navbar
3. Fill out signup form:
   - Name: Test User
   - Email: test@example.com
   - Password: TestPassword123
   - Confirm Password: TestPassword123
   - Experience Level: Beginner
   - Programming Languages: Select at least one (e.g., Python)
   - Areas of Interest: Select at least one (e.g., Artificial Intelligence)
   - Hardware Skill Level: Beginner
   - Professional Role: Student
   - Organization: (optional) Test University
4. Click "Create Account"

**Expected Results**:

- ✅ Form validates in real-time (email format, password match)
- ✅ Success message appears: "Welcome to Physical AI!"
- ✅ User automatically logged in
- ✅ Redirected to `/docs/introduction/what-is-physical-ai`
- ✅ Navbar shows user name/email and "Logout" button
- ✅ Database has new user record with all custom fields

**Verify in Database**:

```sql
SELECT id, email, name, experience_level, professional_role, organization
FROM "user"
WHERE email = 'test@example.com';
```

---

### Test 2: User Signin Flow

**Objective**: Verify existing users can sign in with correct credentials.

**Steps**:

1. If logged in, click "Logout"
2. Navigate to `http://localhost:3000`
3. Click "Sign In" button in navbar
4. Enter credentials:
   - Email: test@example.com
   - Password: TestPassword123
5. Click "Sign In"

**Expected Results**:

- ✅ Successful signin
- ✅ Redirected to `/docs/introduction/what-is-physical-ai`
- ✅ Navbar shows user name/email and "Logout" button
- ✅ Session persists across page navigation

---

### Test 3: Invalid Credentials

**Objective**: Verify error handling for incorrect login attempts.

**Steps**:

1. Navigate to signin page
2. Enter incorrect credentials:
   - Email: test@example.com
   - Password: WrongPassword123
3. Click "Sign In"

**Expected Results**:

- ✅ Error message appears: "Invalid email or password"
- ✅ User remains on signin page
- ✅ No user enumeration (doesn't reveal which field is wrong)

---

### Test 4: User Logout Flow

**Objective**: Verify users can securely log out.

**Steps**:

1. Sign in as test user
2. Verify navbar shows user name and "Logout" button
3. Click "Logout"

**Expected Results**:

- ✅ Session terminated
- ✅ Redirected to homepage
- ✅ Navbar shows "Sign In" and "Sign Up" buttons
- ✅ Cannot access protected features without re-authenticating

---

### Test 5: Session Persistence

**Objective**: Verify sessions persist across browser sessions.

**Steps**:

1. Sign in as test user
2. Navigate to several documentation pages
3. Close browser completely
4. Reopen browser and navigate to `http://localhost:3000`

**Expected Results**:

- ✅ User still logged in (navbar shows user info)
- ✅ Session cookie still valid
- ✅ Can navigate site without re-authenticating

---

### Test 6: Duplicate Email Prevention

**Objective**: Verify users cannot create duplicate accounts.

**Steps**:

1. Try to sign up with existing email (test@example.com)
2. Fill out form with same email but different details
3. Click "Create Account"

**Expected Results**:

- ✅ Error message appears
- ✅ User not created in database
- ✅ Original account unaffected

---

### Test 7: FastAPI Integration

**Objective**: Verify backend API accepts and logs authenticated user data.

**Steps**:

1. Sign in as test user
2. Open browser DevTools (F12) → Network tab
3. Use the chatbot widget to ask a question
4. Check request headers in Network tab

**Expected Results**:

- ✅ Request includes `Authorization: Bearer <token>` header
- ✅ Backend logs show authenticated user info
- ✅ Query logged with user metadata in database

**Verify Backend Logs**:

Look for log entry like:

```
Received query from authenticated user: test@example.com (experience: beginner, role: student)
```

**Manual API Test**:

```bash
# 1. Sign in and get token (check browser cookies or network tab)
TOKEN="<your-jwt-token>"

# 2. Query backend with authentication
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{"q": "What is physical AI?", "top_k": 5}'
```

---

### Test 8: Mobile Responsiveness

**Objective**: Verify auth pages work on mobile devices.

**Steps**:

1. Open browser DevTools (F12)
2. Toggle device toolbar (Ctrl+Shift+M)
3. Set viewport to 320px width (iPhone SE)
4. Navigate through signup/signin flows

**Expected Results**:

- ✅ Forms render correctly at 320px width
- ✅ All buttons clickable
- ✅ Text readable
- ✅ No horizontal scrolling required

---

### Test 9: Session Expiration

**Objective**: Verify expired sessions require re-authentication.

**Steps**:

1. Sign in as test user
2. Manually expire session in database:

```sql
UPDATE "session"
SET "expiresAt" = NOW() - INTERVAL '1 day'
WHERE "userId" = (SELECT id FROM "user" WHERE email = 'test@example.com');
```

3. Refresh browser
4. Try to navigate site

**Expected Results**:

- ✅ Session automatically cleared
- ✅ User logged out
- ✅ Navbar shows "Sign In" button

---

### Test 10: Password Validation

**Objective**: Verify password requirements are enforced.

**Steps**:

1. Navigate to signup page
2. Try passwords shorter than 8 characters
3. Try mismatched password confirmation

**Expected Results**:

- ✅ Error for passwords < 8 characters
- ✅ Error for mismatched passwords
- ✅ Real-time validation feedback
- ✅ Submit button disabled until valid

---

## Automated Testing

### Backend API Tests

Test JWT validation in FastAPI:

```bash
cd backend
pytest tests/test_jwt_validation.py -v
```

### Frontend Component Tests

(If implemented)

```bash
npm test -- --watch=false
```

---

## Performance Testing

### Auth Service Response Time

Test auth endpoint performance:

```bash
# Install Apache Bench
# Ubuntu: sudo apt-get install apache2-utils
# macOS: brew install apache2-utils

# Test signup endpoint
ab -n 100 -c 10 -p signup_payload.json -T application/json \
  http://localhost:3001/api/auth/signup/email

# Test signin endpoint
ab -n 100 -c 10 -p signin_payload.json -T application/json \
  http://localhost:3001/api/auth/signin/email
```

**Success Criteria**:

- ✅ Auth endpoints respond in < 500ms (p95)
- ✅ No errors under normal load
- ✅ Database handles concurrent requests

---

## Troubleshooting Common Issues

### Issue: "DATABASE_URL not set"

**Solution**: Check `auth-service/.env` file exists and contains valid `DATABASE_URL`.

### Issue: CORS errors in browser console

**Solution**: Verify `CORS_ORIGINS` in `auth-service/.env` includes `http://localhost:3000`.

### Issue: "Session not found" errors

**Solution**: Clear browser cookies and sign in again.

### Issue: Signup form doesn't submit

**Solution**: Check browser console for validation errors. Ensure all required fields filled.

### Issue: Backend doesn't recognize JWT token

**Solution**: Verify `BETTER_AUTH_JWT_SECRET` matches in both `auth-service/.env` and `backend/.env`.

---

## Security Verification

Run these checks before production deployment:

1. **Password Hashing**: Verify passwords are hashed in database (not plain text)

```sql
SELECT password FROM "account" LIMIT 1;
-- Should show hashed value like: $2a$10$...
```

2. **Cookie Security**: Check cookies in browser DevTools → Application → Cookies

- ✅ `httpOnly` flag set
- ✅ `secure` flag set (in production)
- ✅ `sameSite` configured

3. **JWT Secret**: Verify secret is not default value

```bash
grep BETTER_AUTH_SECRET auth-service/.env
# Should NOT be: "your-256-bit-secret-key-change-this-in-production"
```

---

## Test Data Cleanup

After testing, clean up test data:

```sql
-- Delete test user
DELETE FROM "session" WHERE "userId" IN (
  SELECT id FROM "user" WHERE email LIKE 'test%'
);

DELETE FROM "account" WHERE "userId" IN (
  SELECT id FROM "user" WHERE email LIKE 'test%'
);

DELETE FROM "user" WHERE email LIKE 'test%';
```

---

## Success Checklist

Before marking authentication as complete, verify:

- [ ] All 10 test scenarios pass
- [ ] Mobile responsiveness confirmed (320px - 1920px)
- [ ] CORS configured correctly
- [ ] Database migrations applied successfully
- [ ] Password hashing verified
- [ ] Session management works correctly
- [ ] FastAPI integration logs user metadata
- [ ] No breaking changes to existing documentation site
- [ ] All environment variables documented
- [ ] Docker Compose setup works
- [ ] Security checklist passed

---

## Next Steps

Once testing is complete:

1. Deploy to staging environment
2. Run full end-to-end tests in staging
3. Load test with realistic traffic
4. Security audit
5. Deploy to production

See `DEPLOYMENT.md` for production deployment instructions.
