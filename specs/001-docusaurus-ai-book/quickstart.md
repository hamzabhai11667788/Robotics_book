# Quick Start Guide: Docusaurus Book for Physical AI

This guide will help you get started with the Docusaurus-based textbook project.

## Prerequisites

Before you begin, ensure you have the following installed:

*   **Node.js**: Version 18.x or later (LTS recommended). You can download it from [nodejs.org](https://nodejs.org/).
*   **npm** (Node Package Manager) or **Yarn**: npm is included with Node.js. Yarn can be installed globally via npm (`npm install -g yarn`).
*   **Git**: For cloning the repository.

## Getting Started

Follow these steps to set up the project locally:

### 1. Clone the Repository

Open your terminal or command prompt and clone the project:

```bash
git clone [REPOSITORY_URL] # Replace with actual repository URL
cd 001-docusaurus-ai-book # Or the appropriate directory name
```

### 2. Install Dependencies

Navigate to the project root directory (where `package.json` is located) and install the Docusaurus dependencies:

```bash
npm install # Or yarn install
```

### 3. Start the Local Development Server

Once dependencies are installed, you can start the development server. This will open the website in your browser and provide live reloading for any changes you make to the content.

```bash
npm start # Or yarn start
```

The website should open in your default browser at `http://localhost:3000`.

### 4. Build the Static Site

To generate a production-ready static build of the website, run the build command:

```bash
npm run build # Or yarn build
```

This will create a `build` directory in your project root, containing all the static HTML, CSS, and JavaScript files ready for deployment to any static web host.

## Contributing

To contribute to the textbook, simply edit the Markdown files located in the `docs/` directory. Your changes will be reflected live when running the development server (`npm start`).
