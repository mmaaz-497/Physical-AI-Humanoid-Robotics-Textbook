import { cors } from 'hono/cors';

// Get allowed origins from environment
const allowedOrigins = process.env.CORS_ORIGINS?.split(',') || ['http://localhost:3000'];

// Create CORS middleware
export const corsMiddleware = cors({
  origin: allowedOrigins,
  credentials: true,
  allowHeaders: ['Content-Type', 'Authorization'],
  allowMethods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  exposeHeaders: ['Content-Length', 'Set-Cookie'],
  maxAge: 600, // 10 minutes
});
