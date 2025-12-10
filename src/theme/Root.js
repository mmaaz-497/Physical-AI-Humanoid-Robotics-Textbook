import React, { useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Root({ children }) {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    // Inject chatbot widget script
    const script = document.createElement('script');

    // Use baseUrl for widget path - works for both GitHub Pages and Vercel
    const baseUrl = process.env.NODE_ENV === 'production'
      ? (window.location.hostname.includes('github.io')
          ? '/Physical-AI-Humanoid-Robotics-Textbook'
          : '')
      : '';
    script.src = `${baseUrl}/widget.js`;

    // Use customField from docusaurus.config.js for API URL
    const apiUrl = siteConfig.customFields?.chatbotApiUrl || 'http://localhost:8000';
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

  return <>{children}</>;
}
