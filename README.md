# Physical AI & Humanoid Robotics Book

Comprehensive guide to Physical AI and Humanoid Robotics built with Docusaurus.

## Overview

This interactive book covers:
- Introduction to Physical AI & Humanoid Robotics
- Development Environment Setup (Workstation, Edge, Cloud)
- ROS 2 Fundamentals
- Digital Twin Simulations (Gazebo, Unity)
- NVIDIA Isaac Platform
- Vision-Language-Action (VLA) Systems
- Humanoid Robot Control & Manipulation

## Prerequisites

- **Node.js**: 18.x or 20.x LTS
- **npm**: 8.x or higher

## Installation

```bash
npm install
```

## Development

Start the development server:

```bash
npm start
```

This command starts a local development server and opens a browser window. Most changes are reflected live without having to restart the server.

## Build

Build the static site:

```bash
npm run build
```

This command generates static content into the `build` directory.

## Deployment

Deploy to GitHub Pages:

```bash
npm run deploy
```

## Project Structure

```
├── docs/                        # Documentation pages
│   ├── 01-introduction/         # Physical AI introduction (5 chapters)
│   ├── 02-setup-guides/         # Setup guides (3 chapters)
│   ├── 03-ros2/                 # ROS 2 tutorials (5 chapters)
│   ├── 04-digital-twin/         # Digital Twin (5 chapters)
│   ├── 05-isaac/                # NVIDIA Isaac (6 chapters)
│   ├── 06-vla-humanoids/        # VLA & Humanoids (7 chapters)
│   └── 07-references/           # References (4 chapters)
├── backend/                     # RAG Chatbot Backend (FastAPI)
│   ├── src/                     # Source code
│   │   ├── routers/             # API endpoints
│   │   ├── services/            # Business logic (Qdrant, OpenAI, DB)
│   │   ├── models/              # Data models (Pydantic, SQLAlchemy)
│   │   └── utils/               # Utilities (logging, exceptions)
│   ├── migrations/              # Database migrations
│   ├── scripts/                 # Helper scripts
│   ├── Dockerfile               # Docker configuration
│   └── requirements.txt         # Python dependencies
├── frontend/                    # RAG Chatbot Frontend Widget
│   ├── src/
│   │   └── widget.js            # Universal JavaScript widget
│   └── README.md                # Widget documentation
├── specs/                       # Feature specifications
│   └── 002-rag-chatbot-integration/
│       ├── spec.md              # Requirements
│       ├── plan.md              # Implementation plan
│       └── tasks.md             # Task list
├── static/                      # Static assets
│   ├── img/                     # Images and diagrams
│   ├── code/                    # Code examples
│   └── data/                    # URDF/SDF models
├── src/                         # Docusaurus source files
│   ├── components/              # React components
│   ├── css/                     # Custom CSS
│   └── pages/                   # Custom pages
├── docusaurus.config.js         # Docusaurus configuration
├── sidebars.js                  # Sidebar configuration
├── package.json                 # Node.js dependencies
└── RAG_CHATBOT_IMPLEMENTATION.md # RAG Chatbot architecture guide
```

## Features

- 7 comprehensive learning modules
- 36 hands-on chapters
- Interactive code examples
- Syntax highlighting for Python, C++, XML, YAML, Bash
- Dark mode support
- Mobile-responsive design
- Collapsible sidebar navigation
- Search functionality
- **AI-Powered RAG Chatbot** - Ask questions about the textbook content with grounded answers

## RAG Chatbot Integration

This project includes a production-ready RAG (Retrieval-Augmented Generation) chatbot that provides intelligent, grounded answers to questions about the textbook content.

### Architecture

- **Backend**: FastAPI with Qdrant vector search and OpenAI GPT-4o-mini
- **Database**: Neon Serverless Postgres for chat logging
- **Frontend**: Universal JavaScript widget (no dependencies)
- **Zero Hallucinations**: All answers strictly grounded in textbook content

### Quick Start

#### Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
cp .env.example .env
# Edit .env with your API keys
uvicorn src.main:app --reload
```

#### Frontend Widget Integration

Add to your Docusaurus site:

```bash
# Copy widget to static folder
cp frontend/src/widget.js docs/static/

# Add to docusaurus.config.js scripts array
scripts: [
  {
    src: '/widget.js',
    async: true,
    'data-api-url': 'http://localhost:8000',
  },
]
```

### Documentation

- **Backend**: See `backend/README.md` for complete API documentation
- **Frontend**: See `frontend/README.md` for widget integration guide
- **Implementation**: See `RAG_CHATBOT_IMPLEMENTATION.md` for architecture details

### Features

✅ **Zero Hallucinations** - Strict grounding in textbook content
✅ **Semantic Search** - Qdrant vector database with similarity threshold
✅ **Citation Links** - Every answer links to source chapters
✅ **Highlighted Text Mode** - Ask questions about selected text
✅ **Chat Logging** - All interactions stored in Postgres
✅ **Production Ready** - Docker, CORS, error handling, health checks

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test locally with `npm start`
5. Submit a pull request

## License

MIT License

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Docusaurus Documentation](https://docusaurus.io/docs)
