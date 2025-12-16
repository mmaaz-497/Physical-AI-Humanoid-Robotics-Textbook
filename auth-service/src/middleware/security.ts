/**
 * Security Middleware for Better Auth Service
 * Implements rate limiting and security headers
 */

import { Context, Next } from 'hono';

// Simple in-memory rate limiter (for production, use Redis)
interface RateLimitEntry {
  count: number;
  resetTime: number;
}

const rateLimitStore = new Map<string, RateLimitEntry>();

// Clean up old entries every 5 minutes
setInterval(() => {
  const now = Date.now();
  for (const [key, entry] of rateLimitStore.entries()) {
    if (entry.resetTime < now) {
      rateLimitStore.delete(key);
    }
  }
}, 5 * 60 * 1000);

/**
 * Rate limiting middleware
 * Default: 10 requests per minute per IP for auth endpoints
 */
export function rateLimiter(options: {
  maxRequests?: number;
  windowMs?: number;
  skipPaths?: string[];
} = {}) {
  const maxRequests = options.maxRequests || 10;
  const windowMs = options.windowMs || 60 * 1000; // 1 minute
  const skipPaths = options.skipPaths || ['/health'];

  return async (c: Context, next: Next) => {
    const path = c.req.path;

    // Skip rate limiting for certain paths
    if (skipPaths.some(skip => path.includes(skip))) {
      return next();
    }

    // Get client IP (handle proxies)
    const ip = c.req.header('x-forwarded-for')?.split(',')[0].trim() ||
                c.req.header('x-real-ip') ||
                'unknown';

    const key = `${ip}:${path}`;
    const now = Date.now();

    let entry = rateLimitStore.get(key);

    if (!entry || entry.resetTime < now) {
      // Create new entry
      entry = {
        count: 1,
        resetTime: now + windowMs,
      };
      rateLimitStore.set(key, entry);
      return next();
    }

    // Increment request count
    entry.count++;

    if (entry.count > maxRequests) {
      // Rate limit exceeded
      const retryAfter = Math.ceil((entry.resetTime - now) / 1000);

      return c.json(
        {
          error: 'Too Many Requests',
          message: 'Rate limit exceeded. Please try again later.',
          retryAfter,
        },
        429,
        {
          'Retry-After': retryAfter.toString(),
          'X-RateLimit-Limit': maxRequests.toString(),
          'X-RateLimit-Remaining': '0',
          'X-RateLimit-Reset': entry.resetTime.toString(),
        }
      );
    }

    // Add rate limit headers
    c.header('X-RateLimit-Limit', maxRequests.toString());
    c.header('X-RateLimit-Remaining', (maxRequests - entry.count).toString());
    c.header('X-RateLimit-Reset', entry.resetTime.toString());

    return next();
  };
}

/**
 * Security headers middleware
 * Adds common security headers to all responses
 */
export function securityHeaders() {
  return async (c: Context, next: Next) => {
    await next();

    // Set security headers
    c.header('X-Content-Type-Options', 'nosniff');
    c.header('X-Frame-Options', 'DENY');
    c.header('X-XSS-Protection', '1; mode=block');
    c.header('Referrer-Policy', 'strict-origin-when-cross-origin');
    c.header('Permissions-Policy', 'geolocation=(), microphone=(), camera=()');

    // Content Security Policy (adjust for your needs)
    if (process.env.NODE_ENV === 'production') {
      c.header(
        'Strict-Transport-Security',
        'max-age=31536000; includeSubDomains; preload'
      );
    }
  };
}

/**
 * Request logging middleware
 * Logs all requests for security monitoring
 */
export function requestLogger() {
  return async (c: Context, next: Next) => {
    const start = Date.now();
    const method = c.req.method;
    const path = c.req.path;
    const ip = c.req.header('x-forwarded-for')?.split(',')[0].trim() ||
                c.req.header('x-real-ip') ||
                'unknown';
    const userAgent = c.req.header('user-agent') || 'unknown';

    await next();

    const duration = Date.now() - start;
    const status = c.res.status;

    // Log request (in production, send to logging service)
    console.log(
      JSON.stringify({
        timestamp: new Date().toISOString(),
        method,
        path,
        status,
        duration: `${duration}ms`,
        ip,
        userAgent,
        // Don't log sensitive data
        service: 'auth-service',
      })
    );

    // Alert on suspicious activity
    if (status === 401 || status === 429) {
      console.warn(
        `[SECURITY] Suspicious activity from IP ${ip}: ${method} ${path} -> ${status}`
      );
    }
  };
}

/**
 * Body size limit middleware
 * Prevents large payload attacks
 */
export function bodySizeLimit(maxBytes: number = 1024 * 100) { // 100KB default
  return async (c: Context, next: Next) => {
    const contentLength = parseInt(c.req.header('content-length') || '0', 10);

    if (contentLength > maxBytes) {
      return c.json(
        {
          error: 'Payload Too Large',
          message: `Request body exceeds maximum size of ${maxBytes} bytes`,
        },
        413
      );
    }

    return next();
  };
}
