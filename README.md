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
├── static/                      # Static assets
│   ├── img/                     # Images and diagrams
│   ├── code/                    # Code examples
│   └── data/                    # URDF/SDF models
├── src/                         # Source files
│   ├── components/              # React components
│   ├── css/                     # Custom CSS
│   └── pages/                   # Custom pages
├── docusaurus.config.js         # Docusaurus configuration
├── sidebars.js                  # Sidebar configuration
└── package.json                 # Node.js dependencies
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
