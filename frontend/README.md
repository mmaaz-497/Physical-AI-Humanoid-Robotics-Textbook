# Physical AI Textbook RAG Chatbot - Frontend Widget

Universal JavaScript widget for embedding the RAG chatbot in any web page, including Docusaurus sites.

## Features

✨ **Universal**: Works in any modern website (React, Vue, vanilla HTML, Docusaurus)
✨ **Highlighted Text Mode**: Automatically detects and uses highlighted text
✨ **Clean UI**: Floating icon + expandable chat panel
✨ **Citation Links**: Clickable sources that navigate to book chapters
✨ **Responsive**: Works on desktop, tablet, and mobile
✨ **Zero Dependencies**: Pure JavaScript, no framework required

## Quick Start

### Option 1: Direct Script Tag (Easiest)

Add this to your HTML `<body>`:

```html
<script src="https://your-cdn.com/widget.js" data-api-url="https://your-api.com"></script>
```

That's it! The widget will appear in the bottom-right corner.

### Option 2: Docusaurus Integration

#### Method A: Root Component (Recommended)

1. Swizzle the Root component:

```bash
cd docs
npm run swizzle @docusaurus/theme-classic Root -- --eject
```

2. Edit `docs/src/theme/Root.js`:

```jsx
import React, { useEffect } from 'react';

export default function Root({ children }) {
  useEffect(() => {
    // Inject chatbot widget script
    const script = document.createElement('script');
    script.src = '/widget.js';  // Or your CDN URL
    script.setAttribute('data-api-url', 'https://your-api-url.com');
    script.async = true;
    document.body.appendChild(script);

    return () => {
      // Cleanup on unmount
      if (script.parentNode) {
        document.body.removeChild(script);
      }
    };
  }, []);

  return <>{children}</>;
}
```

3. Copy `widget.js` to `docs/static/`:

```bash
cp frontend/src/widget.js docs/static/
```

4. Configure API URL in `docusaurus.config.js`:

```js
module.exports = {
  // ... other config
  customFields: {
    chatbotApiUrl: process.env.CHATBOT_API_URL || 'http://localhost:8000',
  },
};
```

#### Method B: Custom HTML (Alternative)

Add to `docusaurus.config.js`:

```js
module.exports = {
  scripts: [
    {
      src: '/widget.js',
      async: true,
      'data-api-url': 'https://your-api-url.com',
    },
  ],
};
```

### Option 3: React Component

```jsx
import { useEffect } from 'react';

function App() {
  useEffect(() => {
    const script = document.createElement('script');
    script.src = '/widget.js';
    script.setAttribute('data-api-url', 'https://your-api-url.com');
    document.body.appendChild(script);

    return () => {
      document.body.removeChild(script);
    };
  }, []);

  return <div>Your app content</div>;
}
```

## Configuration

### API URL

The widget reads the API URL from the `data-api-url` attribute:

```html
<script src="/widget.js" data-api-url="https://api.example.com"></script>
```

Default: `http://localhost:8000` (for local development)

### Customization

Edit `src/widget.js` to customize:

- Colors: Change `#2563eb` to your brand color
- Position: Modify `bottom` and `right` values
- Size: Adjust widget dimensions
- Icons: Replace SVG paths with your own

## Usage

### Basic Questions

1. Click the blue chat icon in the bottom-right corner
2. Type your question (max 500 characters)
3. Press Enter or click "Send"
4. View the answer with clickable citations

### Highlighted Text Mode

1. **Highlight** any text on the page
2. Click the chat icon
3. Ask a question about the highlighted text
4. The widget will use only that text as context (faster, more precise)

Example:
- Highlight: "ROS 2 nodes are independent processes..."
- Ask: "What are nodes?"
- Answer uses ONLY the highlighted text

### Citation Links

- Click any citation link in the answer
- Automatically navigates to the source chapter
- Scrolls to the relevant section

## Building for Production

### Option 1: Use as-is

The widget is pure JavaScript and works without compilation.

### Option 2: Minify with webpack

1. Install dependencies:

```bash
cd frontend
npm install
```

2. Create `webpack.config.js`:

```js
const path = require('path');

module.exports = {
  entry: './src/widget.js',
  output: {
    path: path.resolve(__dirname, 'build'),
    filename: 'widget.min.js',
  },
  mode: 'production',
};
```

3. Build:

```bash
npm run build
```

Output: `frontend/build/widget.min.js` (<50KB gzipped)

## Troubleshooting

### Widget Not Appearing

- Check browser console for errors
- Verify script tag is in `<body>`, not `<head>`
- Ensure API URL is correct
- Check CORS is configured on backend

### CORS Errors

Add your domain to backend `.env`:

```env
CORS_ORIGINS=http://localhost:3000,https://yourdomain.com
```

### API Timeout

- Check backend is running: `curl https://your-api.com/health`
- Verify network connectivity
- Check browser DevTools Network tab for failed requests

### Highlighted Text Not Working

- Ensure text is actually selected before asking
- Try selecting text, then clicking the widget icon
- Selection must be < 5000 characters

## Styling

The widget uses scoped CSS to avoid conflicts. To customize:

1. Edit colors in `applyStyles()` method
2. Change dimensions in `.chatbot-panel`
3. Modify animations in CSS

Example - Change brand color:

```js
// In widget.js, find and replace:
background: #2563eb;  // Blue
// With:
background: #10b981;  // Green
```

## Browser Support

- ✅ Chrome 90+
- ✅ Firefox 88+
- ✅ Safari 14+
- ✅ Edge 90+
- ✅ Mobile browsers (iOS Safari, Chrome Mobile)

## Performance

- Widget JavaScript: ~15KB (unminified), <5KB (minified + gzipped)
- Initial load: <100ms
- API calls: Typically 2-5 seconds
- Memory usage: <2MB

## Security

- ✅ No cookies or tracking
- ✅ No local storage (optional session tracking can be added)
- ✅ All API calls use HTTPS in production
- ✅ Input validation (max lengths enforced)

## Accessibility

- ✅ Keyboard navigation (Tab to focus, Enter to send, Esc to close)
- ✅ ARIA labels for screen readers
- ✅ High contrast mode support
- ✅ Focus indicators

## Advanced Usage

### Custom Initialization

```html
<script src="/widget.js"></script>
<script>
  // Override default API URL
  window.chatbotWidget = new ChatbotWidget('https://custom-api.com');
</script>
```

### Event Hooks

```js
// Listen for widget open/close
document.addEventListener('chatbot:opened', () => {
  console.log('Widget opened');
});

document.addEventListener('chatbot:closed', () => {
  console.log('Widget closed');
});
```

(Note: Add these events to widget.js if needed)

## Deployment Checklist

- [ ] Copy widget.js to static/ or CDN
- [ ] Update data-api-url to production backend URL
- [ ] Verify CORS is configured for your domain
- [ ] Test on all major browsers
- [ ] Test on mobile devices
- [ ] Check performance (Lighthouse score)
- [ ] Monitor error logs

## Support

For issues:
1. Check browser console for errors
2. Verify backend API is accessible
3. Test with curl: `curl -X POST https://api.com/api/query -H "Content-Type: application/json" -d '{"q":"test"}'`
4. Review Network tab in DevTools

## License

Part of the Physical AI & Humanoid Robotics textbook project.
