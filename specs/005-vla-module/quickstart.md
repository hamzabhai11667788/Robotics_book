# Quickstart: Vision-Language-Action (VLA) Module

## Overview
This quickstart guide will help you set up and run the Vision-Language-Action module documentation locally on your machine.

## Prerequisites
- Node.js (version 18 or higher)
- npm (version 9 or higher) or yarn
- Git
- Basic command-line knowledge
- Access to OpenAI API for Whisper examples (optional for testing)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/[your-repo]/Robotics_book.git
cd Robotics_book
```

### 2. Install Dependencies
```bash
npm install
```
or if using yarn:
```bash
yarn install
```

### 3. Run the Documentation Locally
```bash
npm run start
```
or with yarn:
```bash
yarn start
```

This will start the development server and open the documentation in your default browser at `http://localhost:3000`.

### 4. Navigate to the VLA Content
Once the server is running, you can access the Vision-Language-Action module documentation at:
`http://localhost:3000/docs/vla-module/voice-to-action-whisper`

## Adding New Content
To add new documentation content:

1. Create a new `.md` file in the `docs/vla-module/` directory
2. Add the new file to the `sidebars.js` configuration
3. Add appropriate frontmatter to the document:
```markdown
---
title: Your Document Title
sidebar_position: [appropriate number]
description: [Brief description of chapter content]
---
```

## Chapter-Specific Setup

### Chapter 1: Voice-to-Action with OpenAI Whisper
For examples involving OpenAI Whisper, you may need:
- Python 3.8+
- OpenAI Python package: `pip install openai`
- An OpenAI API key

### Chapter 2: LLM-Based Cognitive Planning for Robotics
For LLM-based examples, you may need:
- Access to an LLM API (OpenAI, Anthropic, etc.) or local LLM
- ROS 2 installation for robotics integration examples
- Appropriate API keys for LLM services

### Chapter 3: Capstone - The Autonomous Humanoid
For the capstone project, you may need:
- Complete robotics simulation environment (Gazebo/Unity)
- Full ROS 2 setup
- All dependencies from previous chapters

## Building for Production
To build the documentation for production deployment:

```bash
npm run build
```

The built static files will be in the `build/` directory, ready for deployment to GitHub Pages.

## Deployment
The documentation is configured to deploy to GitHub Pages automatically. After merging to the main branch, the site will be updated.

To deploy manually:
```bash
npm run deploy
```

## Troubleshooting
- If you encounter issues with dependencies, try clearing the cache: `npm cache clean --force`
- If the site doesn't load properly, try clearing your browser cache
- Make sure you're using the required Node.js version (18+)
- For Whisper examples, ensure your API key is properly configured in a .env file