import { Hono } from 'hono';
import { auth } from './lib/auth.js';
import { corsMiddleware } from './middleware/cors.js';
import { rateLimiter, securityHeaders, requestLogger, bodySizeLimit } from './middleware/security.js';

// Create Hono app
const app = new Hono();

// Apply security middleware (order matters!)
app.use('*', requestLogger());
app.use('*', securityHeaders());
app.use('*', corsMiddleware);
app.use('*', bodySizeLimit(1024 * 200)); // 200KB max body size

// Apply rate limiting to auth endpoints only (more strict)
app.use('/api/auth/signup/**', rateLimiter({ maxRequests: 5, windowMs: 60 * 1000 })); // 5 signups per minute
app.use('/api/auth/signin/**', rateLimiter({ maxRequests: 10, windowMs: 60 * 1000 })); // 10 signins per minute
app.use('/api/auth/**', rateLimiter({ maxRequests: 30, windowMs: 60 * 1000 })); // 30 requests per minute for other endpoints

// Health check endpoint
app.get('/health', (c) => {
  return c.json({ status: 'ok', service: 'auth-service', timestamp: new Date().toISOString() });
});

// Mount Better Auth routes at /api/auth
app.on(['POST', 'GET'], '/api/auth/**', (c) => {
  return auth.handler(c.req.raw);
});

// 404 handler
app.notFound((c) => {
  return c.json({ error: 'Not Found', path: c.req.path }, 404);
});

// Error handler
app.onError((err, c) => {
  console.error(`Error: ${err.message}`, err);
  return c.json(
    {
      error: 'Internal Server Error',
      message: process.env.NODE_ENV === 'development' ? err.message : 'Something went wrong',
    },
    500
  );
});

// Start server
const port = parseInt(process.env.PORT || '3001');

console.log(`🚀 Auth service starting on port ${port}...`);
console.log(`📍 Base URL: ${process.env.BETTER_AUTH_URL || 'http://localhost:3001'}`);
console.log(`🔐 CORS origins: ${process.env.CORS_ORIGINS || 'http://localhost:3000'}`);
console.log(`🛡️  Security features enabled: rate limiting, security headers, request logging`);
console.log(`⚡ Rate limits: 5 signups/min, 10 signins/min, 30 other/min`);

export default {
  port,
  fetch: app.fetch,
};
