<!-- SYNC IMPACT REPORT
Version change: 1.0.0 -> 1.0.1
Modified principles: N/A
Added sections: Integrated RAG Chatbot Requirements
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
Follow-up TODOs: None
-->
# Textbook for Teaching Physical AI & Humanoid Robotics Course Constitution

## Core Principles

### Accuracy
All AI, robotics, and humanoid concepts must be technically correct and based on authoritative sources. All content must be factually accurate and scientifically sound. Rationale: Ensures credibility and educational value for students learning advanced robotics concepts.

### Clarity
Content must be easy to read and understand for intermediate technical learners. Complex concepts should be broken down into digestible explanations with clear examples. Rationale: Facilitates effective learning and comprehension for the target audience.

### Consistency
Uniform terminology, style, and code examples must be maintained across all chapters. All chapters should follow the same structural format and presentation style. Rationale: Creates a cohesive learning experience and avoids confusion from inconsistent terminology.

### Reproducibility
Code snippets and examples should be executable and verifiable by readers. All practical examples must be tested and confirmed to work as described. Rationale: Enables hands-on learning and validates the technical accuracy of the content.

### Docusaurus Compatibility
All content must be in Markdown format fully compatible with Docusaurus documentation framework. Proper formatting, cross-references, and navigation structures must be maintained. Rationale: Ensures proper integration with the chosen documentation platform.

### Educational Focus
Content must prioritize pedagogical effectiveness over comprehensive coverage. Each chapter should have clear learning objectives, examples, and summaries. Rationale: Maintains focus on the educational mission of the textbook.

## Content Standards

Chapter structure: Every chapter must follow the Introduction → Concepts → Examples → Summary format. Code blocks must be properly fenced with appropriate language tags. Diagrams and illustrations must be included where necessary with clear references. All references must cite authoritative sources with priority given to peer-reviewed and credible materials. Writing style must be concise, engaging, and professional.

## Development Constraints

Chapter length must be between 1000–1500 words. Each module must contain exactly three chapters. Content must be generated only for the specified module without using placeholders. All content must maintain both technical correctness and readability for the target audience.

## Success Criteria

The project will be successful if it produces accurate, high-quality technical content in fully Docusaurus-ready Markdown chapters with consistent style, terminology, and formatting. Chapters must be clear, structured, and self-contained to be suitable for learning.

## Governance

This constitution governs all aspects of the textbook development process. All contributors must adhere to these principles when creating content. Any deviations must be documented and justified. Amendments to this constitution require explicit approval from project leadership and must be recorded with version tracking. All reviews must verify compliance with these principles. Version: 1.0.0 | Ratified: 2025-12-20 | Last Amended: 2025-12-21

## Integrated RAG Chatbot Requirements

### Purpose
- Describe an integrated Retrieval-Augmented Generation (RAG) chatbot embedded within the published book.
- The chatbot must answer user questions strictly based on the book's content.
- It must support answering questions using only user-selected text when provided.

### Technologies
- OpenAI Agents / ChatKit SDKs for conversational logic
- FastAPI for backend API services
- Neon Serverless Postgres for structured metadata and session storage
- Qdrant Cloud (Free Tier) for vector storage and semantic retrieval

### Behavioral Rules
- The chatbot must not hallucinate or answer beyond available book content.
- All answers must be grounded in retrieved passages.
- When user-selected text is provided, responses must rely exclusively on that text for generating answers.