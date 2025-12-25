# Quickstart: AI-Robot Brain (NVIDIA Isaac™)

## Overview
This quickstart guide will help you set up and run the AI-Robot Brain documentation locally on your machine.

## Prerequisites
- Node.js (version 18 or higher)
- npm (version 9 or higher) or yarn
- Git
- Basic command-line knowledge

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

### 4. Navigate to the AI-Robot Brain Content
Once the server is running, you can access the AI-Robot Brain documentation at:
`http://localhost:3000/docs/ai-robot-brain/isaac-sim-synthetic-data`

## Adding New Content
To add new documentation content:

1. Create a new `.md` file in the `docs/ai-robot-brain/` directory
2. Add the new file to the `sidebars.js` configuration
3. Add appropriate frontmatter to the document:
```markdown
---
title: Your Document Title
sidebar_position: [appropriate number]
---
```

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

## NVIDIA Isaac Setup
For the content in this module, you may also want to set up the NVIDIA Isaac ecosystem:

### Isaac Sim Installation
- Install Isaac Sim from NVIDIA developer portal
- Follow the installation instructions for your OS
- Verify installation by launching a basic simulation

### Isaac ROS Setup
- Install ROS2 Humble Hawksbill or later
- Install Isaac ROS packages from NVIDIA
- Follow the setup instructions in the Isaac ROS chapter

### Nav2 Setup
- Install Navigation2 packages for ROS2
- Configure for humanoid robot navigation
- Follow the setup instructions in the Nav2 chapter