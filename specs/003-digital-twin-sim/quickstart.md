# Quickstart: Digital Twin Simulation (Gazebo & Unity)

## Overview
This quickstart guide will help you set up and run the Digital Twin Simulation documentation locally on your machine.

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

### 4. Navigate to the Digital Twin Content
Once the server is running, you can access the Digital Twin Simulation documentation at:
`http://localhost:3000/docs/digital-twin/physics-simulation-gazebo`

## Adding New Content
To add new documentation content:

1. Create a new `.md` file in the `docs/digital-twin/` directory
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

## Simulation Environment Setup
For the content in this module, you may also want to set up the simulation environments:

### Gazebo Setup
- Install ROS/ROS2 (Robot Operating System)
- Install Gazebo simulation environment
- Follow the setup instructions in the Physics Simulation with Gazebo chapter

### Unity Setup
- Install Unity Hub and Unity Editor (2021.3 LTS or later recommended)
- Install required packages for robotics simulation
- Follow the setup instructions in the Digital Twins & HRI in Unity chapter