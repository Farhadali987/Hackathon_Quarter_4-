---
title: Quickstart Guide for Judges
---

# Quickstart Guide for Judges

## Welcome to the AI Textbook for Physical AI & Humanoid Robotics

Thank you for evaluating our AI-native textbook project for Panaversity Hackathon I. This guide will help you quickly assess our submission and verify that it meets all the requirements.

## Project Overview

Our project implements an AI-native textbook for learning Physical AI and Humanoid Robotics with the following key features:

- **AI-native textbook**: Interactive learning with RAG chatbot
- **Comprehensive content**: Covers ROS 2, Gazebo, NVIDIA Isaac, VLA, and more
- **RAG chatbot**: Answers questions based only on indexed textbook content
- **Personalization**: Adapts to different learning styles and needs
- **Urdu translation**: Preserves technical terminology while making content accessible
- **Hardware & lab guidance**: Practical exercises for hands-on learning

## Quick Evaluation Steps

### 1. Verify the Repository Structure (2 minutes)

Navigate to the project repository and verify:

```
├── backend/                 # FastAPI backend
│   ├── src/
│   ├── requirements.txt
│   └── alembic/
├── frontend/               # Docusaurus v3 frontend
│   ├── src/
│   ├── docs/              # Textbook content
│   ├── docusaurus.config.js
│   └── package.json
├── rag-backend/            # Qdrant-based RAG system
│   ├── src/
│   └── requirements.txt
└── specs/001-ai-textbook-physical-ai/
    ├── spec.md
    ├── plan.md
    ├── tasks.md
    └── contracts/
```

### 2. Check Textbook Content (3 minutes)

Browse the textbook content in `frontend/docs/textbook/`:

- [ ] Chapter 1: Introduction to Physical AI & Humanoid Robotics
- [ ] Chapter 2: ROS 2 (Robot Operating System)
- [ ] Chapter 3: Digital Twin in Robotics
- [ ] Chapter 4: NVIDIA Isaac Platform
- [ ] Chapter 5: Vision-Language-Action (VLA) Models
- [ ] Chapter 6: Humanoid Development
- [ ] Chapter 7: Conversational Robotics
- [ ] Chapter 8: Capstone Hardware Project

### 3. Verify Technology Stack Compliance (3 minutes)

Confirm the project uses the required technologies:

- [ ] **Docusaurus v3**: Check `frontend/package.json` for Docusaurus version
- [ ] **FastAPI**: Check `backend/requirements.txt` for FastAPI dependency
- [ ] **Qdrant Cloud**: Check `rag-backend/requirements.txt` for qdrant-client
- [ ] **Neon Postgres**: Configuration in backend settings
- [ ] **Urdu translation**: Check `frontend/i18n/ur/` directory

### 4. Test the RAG Chatbot (5 minutes)

1. Start the backend service:
   ```bash
   cd backend
   pip install -r requirements.txt
   uvicorn src.main:app --reload
   ```

2. Verify the chatbot endpoint works:
   ```bash
   curl -X POST http://localhost:8000/api/chatbot/query \
     -H "Content-Type: application/json" \
     -d '{"query": "What is ROS 2?", "sessionId": "test-session"}'
   ```

3. Confirm the response is based on textbook content, not hallucinated

### 5. Check Content Ingestion Pipeline (5 minutes)

1. Look for the content ingestion pipeline in `rag-backend/src/`
2. Verify it processes textbook content and stores embeddings in Qdrant
3. Check that the RAG system only uses indexed textbook content

### 6. Verify Urdu Translation (3 minutes)

1. Check the Urdu translation directory: `frontend/i18n/ur/`
2. Verify technical terminology is preserved in translations
3. Confirm the translation functionality works in the frontend

### 7. Test Lab Guidance Features (4 minutes)

1. Check the lab guides in `frontend/docs/lab-guides/`
2. Verify both hardware and cloud lab guidance content exists
3. Test the lab guidance API endpoints:
   ```bash
   curl http://localhost:8000/api/labs
   ```

### 8. Validate Deployment Configuration (3 minutes)

Check for deployment configuration supporting GitHub Pages or Vercil:

- [ ] GitHub Pages configuration in `frontend/docusaurus.config.js`
- [ ] Vercel configuration in `frontend/vercel.json` (if applicable)
- [ ] Build scripts in `frontend/package.json`

### 9. Review Personalization Features (2 minutes)

1. Check for personalization options in the user profile system
2. Verify adaptive learning path functionality exists
3. Look for educator dashboard in frontend

## Technical Requirements Verification

### Performance Goals
- [ ] Support 1000+ concurrent users
- [ ] <200ms p95 for chatbot responses
- [ ] <3s page load times

### Security & Quality
- [ ] Input validation for all API endpoints
- [ ] Rate limiting for chatbot API
- [ ] Proper error handling and logging
- [ ] Authentication framework implemented

### Code Quality
- [ ] Clean, production-ready code
- [ ] Comprehensive tests included
- [ ] Proper documentation
- [ ] Meaningful commit history

## Evaluation Criteria

### Base Requirements (Must Have)
- [ ] Complete textbook on Physical AI & Humanoid Robotics
- [ ] Working RAG chatbot that answers from indexed content only
- [ ] Deployment on GitHub Pages or Vercel
- [ ] Use of Docusaurus v3 and FastAPI

### Bonus Requirements (Nice to Have)
- [ ] Urdu translation with preserved technical terminology
- [ ] Personalization features
- [ ] Hardware and cloud lab guidance
- [ ] Capstone project content

## Quick Demo Script

To quickly demonstrate the key features:

1. Show the textbook interface in the frontend
2. Demonstrate the RAG chatbot answering a question about ROS 2
3. Show the Urdu translation functionality
4. Display the lab guidance content
5. Highlight the personalization options

## Known Issues & Limitations

- The full simulation environment requires significant computational resources
- Some advanced features may require additional API keys for full functionality
- Hardware lab components are documented but require physical setup

## Support & Contact

If you have any questions during evaluation:
- Check the `README.md` for setup instructions
- Review the `specs/001-ai-textbook-physical-ai/` directory for detailed documentation
- Contact the development team through the repository issues

## Conclusion

This AI-native textbook project demonstrates a comprehensive solution for learning Physical AI and Humanoid Robotics. It meets all base requirements and includes several bonus features. The RAG chatbot ensures content integrity, the multilingual support makes it accessible, and the lab guidance provides practical learning experiences.

Thank you for your time in evaluating our submission!