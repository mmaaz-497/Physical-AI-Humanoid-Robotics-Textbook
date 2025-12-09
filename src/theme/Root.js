import React, { useEffect } from 'react';

export default function Root({ children }) {
  useEffect(() => {
    // Inject chatbot widget script
    const script = document.createElement('script');
    script.src = '/Physical-AI-Humanoid-Robotics-Textbook/widget.js';
    script.setAttribute('data-api-url', 'http://localhost:8000');
    script.async = true;

    // Add to document
    document.body.appendChild(script);

    // Log for debugging
    console.log('Chatbot widget script injected');

    return () => {
      // Cleanup on unmount
      if (script.parentNode) {
        document.body.removeChild(script);
      }
    };
  }, []);

  return <>{children}</>;
}
