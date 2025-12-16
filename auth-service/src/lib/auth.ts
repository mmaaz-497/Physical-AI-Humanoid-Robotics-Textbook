import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db/client.js';

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
  }),

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    requireEmailVerification: false, // As per requirement - immediate access
  },

  // User configuration with custom fields
  user: {
    additionalFields: {
      experienceLevel: {
        type: 'string',
        required: true,
        input: true,
        defaultValue: 'beginner',
      },
      professionalRole: {
        type: 'string',
        required: true,
        input: true,
        defaultValue: 'student',
      },
      roleOther: {
        type: 'string',
        required: false,
        input: true,
      },
      organization: {
        type: 'string',
        required: false,
        input: true,
      },
    },
  },

  // Session configuration
  session: {
    expiresIn: parseInt(process.env.SESSION_EXPIRES_IN || '604800'), // 7 days in seconds
    updateAge: parseInt(process.env.SESSION_UPDATE_AGE || '86400'), // Update every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 300, // 5 minutes
    },
  },

  // Security settings
  advanced: {
    generateId: () => crypto.randomUUID(),
    crossSubDomainCookies: {
      enabled: false,
    },
    useSecureCookies: process.env.NODE_ENV === 'production',
  },

  // Base URL configuration
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3001',

  // Trust proxy for production deployments
  trustedOrigins: process.env.CORS_ORIGINS?.split(',') || ['http://localhost:3000'],

  // Secret for signing tokens
  secret: process.env.BETTER_AUTH_SECRET,
});

// Export the auth instance
export type Auth = typeof auth;
