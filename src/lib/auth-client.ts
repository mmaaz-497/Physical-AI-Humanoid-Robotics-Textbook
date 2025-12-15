/**
 * Better Auth Client Configuration for Docusaurus
 * This client communicates with the auth-service for authentication
 */

import { createAuthClient } from 'better-auth/client';

// Get auth service URL from environment or default to localhost
const authServiceUrl =
  typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:3001'
    : process.env.AUTH_SERVICE_URL || 'https://your-auth-service.vercel.app';

export const authClient = createAuthClient({
  baseURL: authServiceUrl,
});

// Export hooks for use in components
export const {
  useSession,
  signIn,
  signUp,
  signOut,
  useActiveOrganization,
} = authClient;
