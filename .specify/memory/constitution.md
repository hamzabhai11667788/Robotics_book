<!--
Sync Impact Report:
- Version change: None -> 1.0.0
- Added sections:
  - Principles: Spec-Driven Development, Test-Driven Development, Small, Atomic Commits, Clear API Contracts, Stateless Services, Secure by Design
  - Development Workflow
  - Governance
- Removed sections: None
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
- Follow-up TODOs: None
-->
# Gemini CLI Agent Project Constitution

## Core Principles

### I. Spec-Driven Development
All features, enhancements, or bug fixes must begin with a clear, written specification document. The specification must define the user stories, requirements, and success criteria before any implementation begins.

### II. Test-Driven Development (NON-NEGOTIABLE)
All code must be developed following a strict Red-Green-Refactor cycle. A failing test must be written to reproduce a bug or define a new feature before the corresponding implementation is written. All tests must pass before code is submitted for review.

### III. Small, Atomic Commits
Code contributions must be made in small, logical, atomic commits. Each commit must represent a single, complete thought and must pass all automated checks. This facilitates easier code review and safer rollbacks.

### IV. Clear API Contracts
All services and components must communicate through well-defined, versioned, and documented API contracts. Any change to a contract requires a version bump and a clear migration plan.

### V. Stateless Services
Services should be designed to be stateless whenever possible. State should be externalized to a dedicated persistence layer (e.g., database, cache), allowing services to be scaled, restarted, and load-balanced without loss of context.

### VI. Secure by Design
Security is not an afterthought. It must be a primary consideration at every stage of the development lifecycle, from specification to deployment. All code must adhere to best practices for authentication, authorization, data validation, and secrets management.

## Development Workflow

All development must follow the established git branching model and pull request process. Code reviews are mandatory for all changes, and must be conducted by at least one other team member.

## Governance

This Constitution is the source of truth for all development practices. Any proposed amendment must be documented in a pull request, reviewed, and approved by the project maintainers. All pull requests must verify compliance with the principles outlined in this document.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06