/**
 * RAG Chatbot Widget for Physical AI Textbook
 * Universal JavaScript widget that can be embedded in any web page
 */

class ChatbotWidget {
    constructor(apiUrl = 'http://localhost:8000') {
        this.apiUrl = apiUrl;
        this.isOpen = false;
        this.conversationHistory = [];
        this.init();
    }

    init() {
        // Create widget HTML
        this.createWidgetHTML();

        // Attach event listeners
        this.attachEventListeners();

        // Apply styles
        this.applyStyles();
    }

    createWidgetHTML() {
        // Create widget container
        const widgetContainer = document.createElement('div');
        widgetContainer.id = 'chatbot-widget-container';
        widgetContainer.innerHTML = `
            <div id="chatbot-widget-icon" class="chatbot-icon" title="Ask a question about the textbook">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor">
                    <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
                </svg>
            </div>

            <div id="chatbot-widget-panel" class="chatbot-panel" style="display: none;">
                <div class="chatbot-header">
                    <h3>Physical AI Textbook Assistant</h3>
                    <button id="chatbot-close-btn" class="chatbot-close">×</button>
                </div>

                <div id="chatbot-messages" class="chatbot-messages">
                    <div class="chatbot-message assistant">
                        <p>👋 Hi! I'm your Physical AI textbook assistant. Ask me questions about the content, or highlight text on the page for context-specific help.</p>
                    </div>
                </div>

                <div class="chatbot-input-container">
                    <textarea
                        id="chatbot-input"
                        class="chatbot-input"
                        placeholder="Ask a question about the textbook..."
                        maxlength="500"
                        rows="2"
                    ></textarea>
                    <button id="chatbot-send-btn" class="chatbot-send-btn">Send</button>
                </div>

                <div id="chatbot-loading" class="chatbot-loading" style="display: none;">
                    <div class="chatbot-spinner"></div>
                    <span>Thinking...</span>
                </div>
            </div>
        `;

        document.body.appendChild(widgetContainer);
    }

    attachEventListeners() {
        // Toggle widget
        document.getElementById('chatbot-widget-icon').addEventListener('click', () => {
            this.toggleWidget();
        });

        document.getElementById('chatbot-close-btn').addEventListener('click', () => {
            this.closeWidget();
        });

        // Send message
        document.getElementById('chatbot-send-btn').addEventListener('click', () => {
            this.sendMessage();
        });

        // Send on Enter (Shift+Enter for new line)
        document.getElementById('chatbot-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.sendMessage();
            }
        });
    }

    toggleWidget() {
        this.isOpen = !this.isOpen;
        const panel = document.getElementById('chatbot-widget-panel');
        const icon = document.getElementById('chatbot-widget-icon');

        if (this.isOpen) {
            panel.style.display = 'flex';
            icon.style.display = 'none';
            document.getElementById('chatbot-input').focus();
        } else {
            panel.style.display = 'none';
            icon.style.display = 'flex';
        }
    }

    closeWidget() {
        this.isOpen = false;
        document.getElementById('chatbot-widget-panel').style.display = 'none';
        document.getElementById('chatbot-widget-icon').style.display = 'flex';
    }

    async sendMessage() {
        const inputEl = document.getElementById('chatbot-input');
        const question = inputEl.value.trim();

        if (!question) return;

        // Clear input
        inputEl.value = '';

        // Get highlighted text (if any)
        const selectionText = this.getSelectedText();

        // Add user message to UI
        this.addMessage(question, 'user');

        // Show loading
        this.showLoading(true);

        try {
            // Call API
            const response = await fetch(`${this.apiUrl}/api/query`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    q: question,
                    top_k: 5,
                    selection_text: selectionText
                }),
            });

            if (!response.ok) {
                throw new Error(`API error: ${response.status}`);
            }

            const data = await response.json();

            // Add assistant message with citations
            this.addMessage(data.answer, 'assistant', data.sources);

        } catch (error) {
            console.error('Chatbot error:', error);
            this.addMessage(
                '❌ Sorry, I encountered an error. Please try again later.',
                'assistant'
            );
        } finally {
            this.showLoading(false);
        }
    }

    getSelectedText() {
        const selection = window.getSelection();
        const text = selection.toString().trim();

        if (text && text.length > 0 && text.length < 5000) {
            return text;
        }
        return null;
    }

    addMessage(text, sender, sources = []) {
        const messagesContainer = document.getElementById('chatbot-messages');

        const messageDiv = document.createElement('div');
        messageDiv.className = `chatbot-message ${sender}`;

        const messageText = document.createElement('p');
        messageText.textContent = text;
        messageDiv.appendChild(messageText);

        // Add citations if present
        if (sources && sources.length > 0) {
            const sourcesDiv = document.createElement('div');
            sourcesDiv.className = 'chatbot-sources';
            sourcesDiv.innerHTML = '<strong>Sources:</strong>';

            const sourcesList = document.createElement('ul');
            sources.forEach(source => {
                const li = document.createElement('li');
                const link = document.createElement('a');
                link.href = source.file_path;
                link.textContent = source.chapter;
                link.title = source.chunk_text;
                li.appendChild(link);
                sourcesList.appendChild(li);
            });

            sourcesDiv.appendChild(sourcesList);
            messageDiv.appendChild(sourcesDiv);
        }

        messagesContainer.appendChild(messageDiv);

        // Scroll to bottom
        messagesContainer.scrollTop = messagesContainer.scrollHeight;

        // Save to history
        this.conversationHistory.push({ sender, text, sources });
    }

    showLoading(show) {
        const loadingEl = document.getElementById('chatbot-loading');
        loadingEl.style.display = show ? 'flex' : 'none';
    }

    applyStyles() {
        // Inject CSS styles
        const style = document.createElement('style');
        style.textContent = `
            #chatbot-widget-container {
                position: fixed;
                bottom: 20px;
                right: 20px;
                z-index: 9999;
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            }

            .chatbot-icon {
                width: 60px;
                height: 60px;
                background: #2563eb;
                border-radius: 50%;
                display: flex;
                align-items: center;
                justify-content: center;
                cursor: pointer;
                box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                transition: transform 0.2s;
                color: white;
            }

            .chatbot-icon:hover {
                transform: scale(1.1);
            }

            .chatbot-panel {
                width: 380px;
                height: 550px;
                background: white;
                border-radius: 12px;
                box-shadow: 0 4px 24px rgba(0,0,0,0.15);
                display: flex;
                flex-direction: column;
                overflow: hidden;
            }

            .chatbot-header {
                background: #2563eb;
                color: white;
                padding: 16px;
                display: flex;
                justify-content: space-between;
                align-items: center;
            }

            .chatbot-header h3 {
                margin: 0;
                font-size: 16px;
                font-weight: 600;
            }

            .chatbot-close {
                background: none;
                border: none;
                color: white;
                font-size: 28px;
                cursor: pointer;
                padding: 0;
                width: 30px;
                height: 30px;
                display: flex;
                align-items: center;
                justify-content: center;
            }

            .chatbot-messages {
                flex: 1;
                overflow-y: auto;
                padding: 16px;
                display: flex;
                flex-direction: column;
                gap: 12px;
            }

            .chatbot-message {
                max-width: 85%;
                padding: 10px 14px;
                border-radius: 12px;
                line-height: 1.4;
            }

            .chatbot-message.user {
                align-self: flex-end;
                background: #2563eb;
                color: white;
            }

            .chatbot-message.assistant {
                align-self: flex-start;
                background: #f3f4f6;
                color: #1f2937;
            }

            .chatbot-message p {
                margin: 0;
                font-size: 14px;
            }

            .chatbot-sources {
                margin-top: 8px;
                padding-top: 8px;
                border-top: 1px solid #d1d5db;
                font-size: 12px;
            }

            .chatbot-sources strong {
                display: block;
                margin-bottom: 4px;
            }

            .chatbot-sources ul {
                margin: 0;
                padding-left: 20px;
            }

            .chatbot-sources a {
                color: #2563eb;
                text-decoration: none;
            }

            .chatbot-sources a:hover {
                text-decoration: underline;
            }

            .chatbot-input-container {
                padding: 16px;
                border-top: 1px solid #e5e7eb;
                display: flex;
                gap: 8px;
            }

            .chatbot-input {
                flex: 1;
                padding: 10px;
                border: 1px solid #d1d5db;
                border-radius: 8px;
                font-size: 14px;
                font-family: inherit;
                resize: none;
            }

            .chatbot-input:focus {
                outline: none;
                border-color: #2563eb;
            }

            .chatbot-send-btn {
                background: #2563eb;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 8px;
                cursor: pointer;
                font-size: 14px;
                font-weight: 600;
            }

            .chatbot-send-btn:hover {
                background: #1d4ed8;
            }

            .chatbot-loading {
                padding: 12px 16px;
                display: flex;
                align-items: center;
                gap: 8px;
                background: #f9fafb;
                border-top: 1px solid #e5e7eb;
                font-size: 14px;
                color: #6b7280;
            }

            .chatbot-spinner {
                width: 16px;
                height: 16px;
                border: 2px solid #e5e7eb;
                border-top-color: #2563eb;
                border-radius: 50%;
                animation: spin 0.6s linear infinite;
            }

            @keyframes spin {
                to { transform: rotate(360deg); }
            }

            @media (max-width: 768px) {
                .chatbot-panel {
                    width: 90vw;
                    height: 70vh;
                }
            }
        `;

        document.head.appendChild(style);
    }
}

// Auto-initialize when script loads
if (typeof window !== 'undefined') {
    // Get API URL from data attribute or default to localhost
    const apiUrl = document.currentScript?.getAttribute('data-api-url') || 'http://localhost:8000';

    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => {
            window.chatbotWidget = new ChatbotWidget(apiUrl);
        });
    } else {
        window.chatbotWidget = new ChatbotWidget(apiUrl);
    }
}
