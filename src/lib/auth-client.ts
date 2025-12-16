/**
 * Better Auth Client Configuration for Docusaurus
 * This client communicates with the auth-service for authentication
 */

import { createAuthClient } from 'better-auth/react';

// Get auth service URL from environment or default to localhost
const getAuthServiceUrl = () => {
  if (typeof window === 'undefined') {
    return 'http://localhost:3001';
  }
  return window.location.hostname === 'localhost'
    ? 'http://localhost:3001'
    : process.env.AUTH_SERVICE_URL || 'https://your-auth-service.vercel.app';
};

export const authClient = createAuthClient({
  baseURL: getAuthServiceUrl(),
});

// Export React hooks from the client
export const useSession = authClient.useSession;
export const signIn = authClient.signIn;
export const signUp = authClient.signUp;
export const signOut = authClient.signOut;
