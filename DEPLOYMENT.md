# Production Deployment Guide

Complete guide for deploying the Physical AI Textbook with Better Auth to production.

## Table of Contents

- [Overview](#overview)
- [Pre-Deployment Checklist](#pre-deployment-checklist)
- [Environment Setup](#environment-setup)
- [Deployment Options](#deployment-options)
- [Post-Deployment](#post-deployment)
- [Monitoring](#monitoring)
- [Backup and Recovery](#backup-and-recovery)

## Overview

The application consists of three main services:

1. **Frontend (Docusaurus)**: Static site deployed to Vercel/Netlify/GitHub Pages
2. **Auth Service (Node.js)**: Deployed to Vercel/Railway/Fly.io
3. **Backend API (FastAPI)**: Deployed to Render/Railway/Fly.io

## Pre-Deployment Checklist

### Security

- [ ] Generate strong `BETTER_AUTH_SECRET` (not the example value)
- [ ] Use strong database passwords
- [ ] Enable SSL/HTTPS for all services
- [ ] Review and update `CORS_ORIGINS` for production domains
- [ ] Enable `secure` cookie flag (automatic in production)
- [ ] Verify rate limiting is enabled
- [ ] Check security headers are set

### Configuration

- [ ] Set `NODE_ENV=production` for auth service
- [ ] Update `BETTER_AUTH_URL` to production URL
- [ ] Update `AUTH_SERVICE_URL` in frontend config
- [ ] Configure proper `SESSION_EXPIRES_IN` (default 7 days is fine)
- [ ] Test all environment variables are set correctly

### Database

- [ ] Database migrations applied successfully
- [ ] Database backups configured
- [ ] Connection pooling configured
- [ ] Database credentials secured (use secrets manager)

### Testing

- [ ] All tests pass locally
- [ ] Manual testing completed (see TESTING_GUIDE.md)
- [ ] Performance testing done
- [ ] Security audit completed

## Environment Setup

### Production Environment Variables

#### Auth Service

```env
# Database (Neon recommended for production)
DATABASE_URL=postgresql://user:password@production-host/db?sslmode=require

# Auth Secret (MUST be different from development!)
BETTER_AUTH_SECRET=<use-openssl-rand-base64-32-to-generate>

# Production URLs
BETTER_AUTH_URL=https://auth.yourdomain.com
PORT=3001
NODE_ENV=production

# CORS (all your frontend domains)
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

# Session config (optional)
SESSION_EXPIRES_IN=604800
SESSION_UPDATE_AGE=86400
```

#### Backend API

```env
# Vector Database
QDRANT_URL=<production-qdrant-url>
QDRANT_API_KEY=<production-api-key>
QDRANT_COLLECTION=physical_ai_textbook

# AI Model
GEMINI_API_KEY=<production-api-key>
GEMINI_MODEL=gemini-2.0-flash

# Database (same as auth service)
NEON_DATABASE_URL=postgresql://user:password@production-host/db?sslmode=require

# Auth (MUST match auth service secret!)
BETTER_AUTH_JWT_SECRET=<same-as-auth-service>

# CORS
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

# API Config
API_HOST=0.0.0.0
API_PORT=8000
```

#### Frontend

```env
# Build-time environment variables
AUTH_SERVICE_URL=https://auth.yourdomain.com
CHATBOT_API_URL=https://api.yourdomain.com
```

## Deployment Options

### Option 1: Vercel (Recommended for Frontend + Auth Service)

#### Deploy Frontend

1. Push code to GitHub
2. Go to [vercel.com](https://vercel.com)
3. Import your repository
4. Configure build settings:
   - Framework: Docusaurus
   - Build Command: `npm run build`
   - Output Directory: `build`
5. Add environment variables:
   - `AUTH_SERVICE_URL`: Your auth service URL
   - `CHATBOT_API_URL`: Your backend API URL
6. Deploy

#### Deploy Auth Service

1. Create `vercel.json` in `auth-service/`:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "src/index.ts",
      "use": "@vercel/node"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "src/index.ts"
    }
  ],
  "env": {
    "NODE_ENV": "production"
  }
}
```

2. Deploy from `auth-service/` directory
3. Add environment variables in Vercel dashboard
4. Get production URL (e.g., `https://auth-service.vercel.app`)

### Option 2: Railway (Full Stack)

#### Deploy All Services

1. Go to [railway.app](https://railway.app)
2. Create new project from GitHub repo
3. Add PostgreSQL service
4. Deploy auth service:
   - Root directory: `auth-service`
   - Start command: `npm start`
   - Add environment variables
5. Deploy backend:
   - Root directory: `backend`
   - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - Add environment variables
6. Deploy frontend:
   - Root directory: `/`
   - Build command: `npm run build`
   - Start command: `npm run serve`

### Option 3: Fly.io

#### Auth Service

1. Install Fly CLI: `curl -L https://fly.io/install.sh | sh`
2. Login: `fly auth login`
3. Create `auth-service/fly.toml`:

```toml
app = "physical-ai-auth"

[build]
  builder = "paketobuildpacks/builder:base"

[env]
  PORT = "3001"
  NODE_ENV = "production"

[[services]]
  http_checks = []
  internal_port = 3001
  protocol = "tcp"

  [[services.ports]]
    handlers = ["http"]
    port = 80

  [[services.ports]]
    handlers = ["tls", "http"]
    port = 443
```

4. Deploy: `fly launch` from `auth-service/`
5. Set secrets: `fly secrets set BETTER_AUTH_SECRET=xxx DATABASE_URL=xxx`

### Option 4: Render

#### Backend API

1. Go to [render.com](https://render.com)
2. New → Web Service
3. Connect GitHub repository
4. Configure:
   - Root directory: `backend`
   - Build command: `pip install -r requirements.txt`
   - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables
6. Deploy

## Database Setup (Neon)

### Create Production Database

1. Go to [neon.tech](https://neon.tech)
2. Create new project (e.g., "physical-ai-production")
3. Enable connection pooling
4. Copy connection string
5. Run migrations:

```bash
# Set DATABASE_URL
export DATABASE_URL="postgresql://..."

# Run migration
psql $DATABASE_URL -f auth-service/migrations/0001_initial_schema.sql
```

### Configure Backups

Neon provides automatic backups. Configure:

1. Point-in-time recovery (PITR): 7 days
2. Branching for testing changes
3. Read replicas if needed for scaling

## Post-Deployment

### 1. Verify Services

Check all services are running:

```bash
# Auth service health
curl https://auth.yourdomain.com/health

# Backend health
curl https://api.yourdomain.com/health

# Frontend
curl https://yourdomain.com
```

### 2. Test Authentication Flow

1. Navigate to production site
2. Click "Sign Up"
3. Create test account
4. Verify email in navbar
5. Test chatbot with authentication
6. Click "Logout"
7. Test "Sign In"

### 3. Configure DNS

Point your domains to deployed services:

```
yourdomain.com           → Frontend (Vercel/Netlify)
www.yourdomain.com       → Frontend (Vercel/Netlify)
auth.yourdomain.com      → Auth Service
api.yourdomain.com       → Backend API
```

### 4. SSL Certificates

Most platforms handle this automatically:

- **Vercel**: Automatic Let's Encrypt
- **Railway**: Automatic SSL
- **Fly.io**: Automatic SSL
- **Render**: Automatic SSL

Verify HTTPS is working:

```bash
curl -I https://yourdomain.com | grep -i "strict-transport-security"
```

### 5. Update CORS

After DNS is configured, update `CORS_ORIGINS`:

**Auth Service:**
```env
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com,https://auth.yourdomain.com
```

**Backend:**
```env
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com
```

## Monitoring

### Application Monitoring

Set up monitoring for:

1. **Uptime**: Use Uptime Robot or Pingdom
2. **Errors**: Sentry or Rollbar
3. **Performance**: New Relic or Datadog
4. **Logs**: Papertrail or Logtail

### Health Check Endpoints

Configure monitoring to check:

- `https://auth.yourdomain.com/health` (every 5 min)
- `https://api.yourdomain.com/health` (every 5 min)
- `https://yourdomain.com` (every 5 min)

### Database Monitoring

Monitor:

- Connection pool usage
- Query performance
- Disk usage
- Backup status

Neon dashboard provides these metrics.

### Security Monitoring

Set up alerts for:

- High rate of 401/403 errors (possible attack)
- 429 errors (rate limiting triggered)
- Failed login attempts
- Database connection errors

## Scaling

### Horizontal Scaling

Most platforms support auto-scaling:

**Vercel/Netlify**: Automatic (CDN-based)

**Railway/Render**:
- Enable auto-scaling based on CPU/memory
- Set min/max instances

**Fly.io**:
```bash
fly scale count 3  # Run 3 instances
fly autoscale set min=1 max=5
```

### Database Scaling

**Neon**:
- Enable read replicas for read-heavy workloads
- Upgrade to higher tier for more compute
- Connection pooling (built-in)

### Caching

Add caching for better performance:

1. **Frontend**: Already cached by CDN (Vercel/Netlify)
2. **API**: Add Redis for session storage and rate limiting
3. **Database**: Enable query result caching

## Backup and Recovery

### Database Backups

**Neon** provides:
- Automatic backups (daily)
- Point-in-time recovery (7-30 days)
- Branch/snapshot creation

**Manual backup:**

```bash
# Backup database
pg_dump $DATABASE_URL > backup-$(date +%Y%m%d).sql

# Restore
psql $DATABASE_URL < backup-20251215.sql
```

### Disaster Recovery Plan

1. **Database failure**:
   - Restore from Neon backup
   - Point-in-time recovery if needed
   - Update connection string if cluster changed

2. **Service failure**:
   - Redeploy from GitHub (automatic rollback)
   - Check error logs
   - Verify environment variables

3. **Complete outage**:
   - Deploy to backup region
   - Update DNS to point to backup
   - Restore database from backup

## Security Best Practices

### Production Checklist

- [ ] All secrets rotated from development values
- [ ] HTTPS enforced (HSTS header)
- [ ] httpOnly and secure cookies enabled
- [ ] Rate limiting active
- [ ] CORS properly configured (not `*`)
- [ ] Security headers set (CSP, X-Frame-Options, etc.)
- [ ] Database uses SSL/TLS
- [ ] No sensitive data in logs
- [ ] Error messages don't leak info
- [ ] Regular security updates applied

### Secrets Management

Use platform secret managers:

**Vercel**: Environment Variables (encrypted)

**Railway**: Secrets (encrypted)

**Fly.io**: `fly secrets set`

**Never**:
- Commit secrets to Git
- Share secrets in Slack/email
- Use default/example secrets in production

## Troubleshooting Production

### Common Issues

#### Users can't sign up

1. Check auth service logs
2. Verify database connection
3. Check CORS headers in browser DevTools
4. Test API directly: `curl -X POST https://auth.yourdomain.com/api/auth/signup/email`

#### CORS errors

1. Verify `CORS_ORIGINS` includes frontend domain
2. Check protocol (http vs https)
3. Verify no trailing slashes in URLs
4. Restart auth service after env changes

#### JWT validation fails

1. Verify `BETTER_AUTH_JWT_SECRET` matches in both services
2. Check token isn't expired
3. Test token manually: `jwt.io`

### Rollback Procedure

If deployment fails:

**Vercel/Netlify**:
1. Go to deployments
2. Click "Promote to Production" on previous version

**Railway/Render**:
1. Redeploy previous commit
2. Or use platform rollback feature

**Fly.io**:
```bash
fly releases
fly releases rollback <version>
```

## Cost Optimization

### Free Tier Usage

Can run entire stack on free tiers:

- **Frontend**: Vercel free tier (100GB bandwidth)
- **Auth Service**: Vercel free tier or Fly.io free tier
- **Backend**: Render free tier (750 hours/month)
- **Database**: Neon free tier (0.5 GB storage, 3 compute hours)

### Scaling Costs

Estimated costs for 10,000 active users:

- **Neon**: ~$20/month (Pro tier)
- **Vercel**: ~$20/month (Pro tier for team features)
- **Backend**: ~$7/month (Render Starter)
- **Auth Service**: Free (Vercel hobby or Fly.io free)

Total: ~$47/month

## Support and Maintenance

### Regular Tasks

**Weekly**:
- Review error logs
- Check uptime reports
- Monitor database growth

**Monthly**:
- Review security alerts
- Update dependencies
- Check cost reports
- Rotate secrets (if policy requires)

**Quarterly**:
- Security audit
- Performance optimization
- Database maintenance
- Backup testing

### Getting Help

- **Platform issues**: Contact support (Vercel, Railway, etc.)
- **Auth issues**: Check Better Auth docs and GitHub issues
- **Database**: Neon support for paid tiers
- **Code issues**: GitHub issues or internal team

## Additional Resources

- [Vercel Documentation](https://vercel.com/docs)
- [Railway Documentation](https://docs.railway.app)
- [Neon Documentation](https://neon.tech/docs)
- [Better Auth Production Guide](https://better-auth.com/docs/production)
- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
