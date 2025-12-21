---
id: 7
title: define-hackathon-rules
stage: constitution
date: 2025-12-21
surface: agent
model: 
feature: none
branch: 001-ai-textbook-physical-ai
user: Engineer Farhad Ali
command: /sp.constitution
labels: [constitution, hackathon, rules, requirements]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

Define strict project rules for Panaversity Hackathon I.

Rules:
- Follow all Hackathon I base + bonus requirements.
- Use Docusaurus v3 for textbook.
- Use FastAPI for backend.
- Use Qdrant Cloud + Neon Postgres for RAG.
- No hallucinated APIs or fake code.
- Beginner-friendly but technically correct.
- RAG chatbot must answer only from indexed content.
- Urdu translation must preserve technical terms.
- Maintain clean git history.

Output a concise constitution document.

## Response snapshot

Updated the project constitution to reflect the specific requirements for Panaversity Hackathon I. The new constitution includes principles for Hackathon Requirements Compliance, Technology Stack Compliance, No Hallucinated APIs or Fake Code, Beginner-Friendly but Technically Correct Content, RAG Content Integrity, Urdu Translation Requirements, and Git History Management.

## Outcome

- ✅ Impact: Created a comprehensive constitution document that defines all the required rules for the Hackathon
- 🧪 Tests: No tests run as this was a documentation task
- 📁 Files: Updated .specify/memory/constitution.md with new principles
- 🔁 Next prompts: Can proceed with creating specifications based on this constitution
- 🧠 Reflection: The constitution clearly defines all the requirements and constraints for the hackathon project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specifications based on this constitution