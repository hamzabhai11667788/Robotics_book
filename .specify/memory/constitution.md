<!-- SYNC IMPACT REPORT:
     Version change: N/A -> 1.0.0
     Modified principles: N/A (new constitution)
     Added sections: All sections (Core Principles, Additional Standards, Constraints, Success Criteria, Governance)
     Removed sections: None
     Templates requiring updates: ✅ plan-template.md, ✅ spec-template.md, ✅ tasks-template.md (no changes needed - templates are generic)
     Follow-up TODOs: None
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-first workflow using Spec-Kit Plus
All development begins with a specification. Features are designed and documented before implementation. This ensures clear requirements, testable outcomes, and reproducible results.

### Technical accuracy from official sources
All content and implementations must be grounded in official documentation and verified sources. No assumptions or speculative implementations are allowed without clear documentation references.

### Clear, developer-focused writing
Documentation and code must prioritize clarity for developers. Complex concepts should be broken down into digestible, actionable information with practical examples.

### Reproducible setup and deployment
All processes must be completely reproducible. Installation, configuration, and deployment procedures must be documented and verified across different environments.

## Additional Standards

Book written with Docusaurus and deployed on GitHub Pages: The primary documentation will be built using Docusaurus and hosted on GitHub Pages for easy access and maintenance.

RAG chatbot grounded only in book content or user-selected text: The chatbot must only reference content from the book or text specifically provided by users, ensuring accuracy and preventing hallucinations.

Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud: The technology stack is standardized to ensure consistency and maintainability.

Runnable, well-documented code: All code examples must be functional and thoroughly documented to enable easy understanding and implementation.

## Constraints

GitHub-based source control: All code and documentation must be managed through GitHub with proper branching and review processes.

No hallucinated responses: The system must not generate content that is not grounded in the provided documentation or user inputs.

End-to-end reproducibility: The entire system from setup to deployment must be reproducible by any developer following the documentation.

## Success Criteria

Live book on GitHub Pages: The documentation must be successfully deployed and accessible via GitHub Pages.

Fully functional embedded RAG chatbot: The chatbot must correctly respond to queries based on the book content.

All specs implemented via Spec-Kit Plus: The entire project must follow the Spec-Kit Plus methodology for consistency.

## Governance

This constitution governs all development activities for the AI/Spec-Driven Book with Embedded RAG Chatbot project. All contributors must adhere to these principles. Amendments require documentation and approval from project maintainers. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-23
