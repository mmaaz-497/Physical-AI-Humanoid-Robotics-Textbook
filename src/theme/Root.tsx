/**
 * Root component wrapper for Docusaurus
 * Provides Better Auth session context to all pages
 */

import React, { useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { authClient } from '../lib/auth-client';

export default function Root({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    // Inject chatbot widget script
    const script = document.createElement('script');

    // Use baseUrl for widget path - works for both GitHub Pages and Vercel
    const baseUrl =
      process.env.NODE_ENV === 'production'
        ? window.location.hostname.includes('github.io')
          ? '/Physical-AI-Humanoid-Robotics-Textbook'
          : ''
        : '';
    script.src = `${baseUrl}/widget.js`;

    // Use customField from docusaurus.config.js for API URL
    const apiUrl =
      (siteConfig.customFields?.chatbotApiUrl as string) || 'http://localhost:8000';
    script.setAttribute('data-api-url', apiUrl);
    script.async = true;

    // Add to document
    document.body.appendChild(script);

    // Log for debugging
    console.log('Chatbot widget script injected with API URL:', apiUrl);

    return () => {
      // Cleanup on unmount
      if (script.parentNode) {
        document.body.removeChild(script);
      }
    };
  }, [siteConfig]);

  // Wrap children with Better Auth SessionProvider
  return <authClient.SessionProvider>{children}</authClient.SessionProvider>;
}
