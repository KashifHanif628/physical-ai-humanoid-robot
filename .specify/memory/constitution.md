<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
List of modified principles: N/A (Initial creation)
Added sections: All principles and sections based on project specifications
Removed sections: None (Initial creation)
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
Follow-up TODOs: None
-->

# AI-Native Book with Integrated RAG Chatbot Constitution

## Core Principles

### Spec-Driven Execution
All implementation must follow Spec-Kit Plus specs; specs must cover book, backend, RAG, and data flow. Every feature starts with a well-defined specification before implementation begins.

### Single Source of Truth
Docusaurus content serves as the single source of truth for the book. All book content must be maintained in Markdown/MDX format in the Docusaurus project.

### Technical Clarity
All code and documentation must prioritize technical clarity for developers. Complex concepts must be explained with runnable code examples and clear explanations.

### Reproducible Production System
The system must be fully reproducible from scratch via specs. All dependencies and configurations must be documented to ensure anyone can rebuild the system.

### Zero Hallucinations
The RAG chatbot must answer from book content only, with strict enforcement of selected-text-only Q&A mode and proper citation of source sections.

### Production-Grade Standards
All implementation must meet production-grade standards including proper error handling, observability, security considerations, and performance optimization.

## Standards and Constraints

**Book**
* Platform: Docusaurus (Markdown/MDX)
* Written via Claude Code
* Deployed to GitHub Pages
* Runnable code examples

**RAG Chatbot**
* Stack: FastAPI + OpenAI Agents/ChatKit SDKs
* Storage: Neon Serverless Postgres + Qdrant Cloud (Free Tier)
* Must: Answer from book content only, Support "selected-text-only" Q&A, Cite source sections

**Constraints**
* Static site deployment
* No external or proprietary data
* Fully documented env vars and setup

## Development Workflow

* All changes must follow Spec-Kit Plus methodology
* Code must be reviewed for adherence to principles
* Testing required for all functionality
* Documentation must be updated with each change

## Governance

All implementation must respect the core principles. The constitution supersedes all other practices. Amendments require proper documentation, approval, and migration plan if needed. All code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17