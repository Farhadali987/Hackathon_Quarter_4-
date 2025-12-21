# Quickstart Guide: AI Textbook for Physical AI & Humanoid Robotics

## Prerequisites

- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Docker (for local development)
- Access to Qdrant Cloud (for vector storage)
- Access to Neon Postgres (for metadata storage)

## Getting Started

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Backend

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your Qdrant and Neon credentials

# Run the backend server
python -m uvicorn src.main:app --reload
```

Backend will be available at `http://localhost:8000`

### 3. Set Up Frontend

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
yarn install

# Set up environment variables
cp .env.example .env
# Edit .env with your backend API URL

# Run the development server
yarn start
```

Frontend will be available at `http://localhost:3000`

### 4. Initialize Content

```bash
# From the backend directory
python scripts/initialize_content.py
```

This script will:
- Create embeddings for all textbook content
- Store embeddings in Qdrant
- Set up initial database schema in Neon Postgres

### 5. Running Tests

```bash
# Backend tests
cd backend
python -m pytest

# Frontend tests
cd frontend
yarn test
```

## Key Endpoints

### Backend (FastAPI)
- API Documentation: `http://localhost:8000/docs`
- Chatbot API: `http://localhost:8000/api/chatbot/query`
- Textbook API: `http://localhost:8000/api/textbook/content`

### Frontend (Docusaurus)
- Main textbook: `http://localhost:3000`
- Chatbot interface: `http://localhost:3000/chat`
- Lab guides: `http://localhost:3000/labs`

## Adding New Content

### Textbook Chapters
1. Create a new markdown file in `frontend/docs/textbook/`
2. Add the file to `frontend/sidebars.js`
3. Run content initialization script to update embeddings

### Lab Guides
1. Create a new markdown file in `frontend/docs/lab-guides/{cloud|hardware}/`
2. Add to appropriate sidebar navigation
3. Update embeddings if needed

## Environment Variables

### Backend (.env)
```
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
BETTER_AUTH_SECRET=your_auth_secret
BETTER_AUTH_URL=http://localhost:8000
```

### Frontend (.env)
```
REACT_APP_BACKEND_API_URL=http://localhost:8000
```

## Deployment

### Frontend (GitHub Pages)
```bash
cd frontend
GIT_USER=<Your GitHub username> yarn deploy
```

### Backend (Vercel)
1. Connect your GitHub repository to Vercel
2. Set environment variables in Vercel dashboard
3. Deploy automatically on push to main branch

### Docker Deployment
```bash
# Build and run all services
docker-compose up --build
```