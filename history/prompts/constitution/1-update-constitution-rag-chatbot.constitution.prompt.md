---
id: 1
title: "Update Constitution RAG Chatbot"
stage: "constitution"
date_iso: "2025-12-21"
surface: "agent"
model: "claude-haiku-4-5-20251001"
feature: "none"
branch: "master"
user: "NBK COMPUTER"
command: "sp.constitution"
labels: ["constitution", "rag", "chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - ".specify/memory/constitution.md"
  - "history/prompts/constitution/1-update-constitution-rag-chatbot.constitution.prompt.md"
tests_yaml: []
---

# Update Constitution RAG Chatbot

## Prompt Text

```
Update the file `/sp.constitution`.

DO NOT remove, rewrite, or modify any existing content.

Append a new section at the end of the document titled:

## Integrated RAG Chatbot Requirements

In this section, clearly document the following:

1. Purpose:
   - Describe an integrated Retrieval-Augmented Generation (RAG) chatbot embedded within the published book.
   - The chatbot must answer user questions strictly based on the book's content.
   - It must support answering questions using only user-selected text when provided.

2. Technologies:
   - OpenAI Agents / ChatKit SDKs for conversational logic
   - FastAPI for backend API services
   - Neon Serverless Postgres for structured metadata and session storage
   - Qdrant Cloud (Free Tier) for vector storage and semantic retrieval

3. Behavioral Rules:
   - The chatbot must not hallucinate or answer beyond available book content.
   - All answers must be grounded in retrieved passages.
   - When user-selected text is provided, responses must rely exclusively
```

## Response Text

Updated the constitution file by appending the new "Integrated RAG Chatbot Requirements" section as requested. The changes included:

1. Updated the sync impact report at the top of the file to reflect the version change from 1.0.0 to 1.0.1
2. Added the new section with three subsections: Purpose, Technologies, and Behavioral Rules
3. Updated the last amended date in the governance section to today's date (2025-12-21)

The new section details the requirements for an integrated RAG chatbot that will be embedded within the published book, specifying its purpose, required technologies, and behavioral rules to prevent hallucination and ensure answers are grounded in the book's content.

## Outcome

Successfully appended the Integrated RAG Chatbot Requirements section to the constitution file while preserving all existing content. The version was updated from 1.0.0 to 1.0.1 to reflect the addition of the new section.

## Evaluation

The constitution file now includes the required section about RAG chatbot requirements. The update followed the specified format and preserved all existing content as requested.