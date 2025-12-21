from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import Optional
from datetime import datetime
import uuid

from src.models.rag_chatbot import RAGChatbotCreate
from src.services.rag import RAGChatbotService
from src.auth import get_current_user, User
from src.utils import log_api_call, log_error, AppException
from src.utils.rate_limiter import rate_limiter
# Note: In a real implementation, the rag_backend would be a separate service
# For this implementation, we'll simulate the RAG functionality directly in the backend
# This is a simplified implementation that doesn't require a separate service

import asyncio
from typing import Dict, List


class RAGAgent:
    async def process_query(self, query: str, session_id: str, user_id: str = None) -> Dict:
        # Simulated response - in a real implementation, this would call the rag-backend service
        # For now, we'll return a simple response based on keywords
        if "ros" in query.lower():
            return {
                "response": "ROS 2 is a flexible framework for developing robot applications. It provides libraries and tools to help software developers create robot applications. It's designed to support multiple programming languages and platforms.",
                "sourceChunks": ["ros2-introduction", "ros2-architecture"]
            }
        elif "humanoid" in query.lower() or "robotics" in query.lower():
            return {
                "response": "Humanoid robotics is the branch of robotics that studies and develops robots with human-like features. These robots have bodily features resembling a human, such as having a head, torso, two arms and two legs.",
                "sourceChunks": ["humanoid-introduction", "humanoid-design"]
            }
        else:
            return {
                "response": "I found some general information about robotics: Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others.",
                "sourceChunks": ["general-robotics"]
            }


rag_agent = RAGAgent()

router = APIRouter()

# Initialize the RAG agent
rag_agent = RAGAgent()


@router.post("/chatbot/query", summary="Submit a query to the RAG chatbot")
async def chatbot_query(
    query_data: RAGChatbotCreate,
    current_user: User = Depends(get_current_user)
):
    """
    Submit a query to the RAG chatbot and receive a response based on textbook content
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call("/api/chatbot/query", "POST", user_id)

        # Rate limiting: Check if user is allowed to make a request
        # Use user ID as identifier, fallback to session ID if available
        identifier = user_id or query_data.sessionId or "anonymous"
        if not rate_limiter.is_allowed(identifier):
            raise HTTPException(
                status_code=429,
                detail="Rate limit exceeded. Please slow down your requests."
            )

        # Validate the query with more detailed checks
        if not query_data.query or len(query_data.query.strip()) == 0:
            raise AppException("Query cannot be empty", status_code=422)

        # Additional validation: check query length
        if len(query_data.query) > 1000:
            raise AppException("Query is too long. Maximum 1000 characters allowed.", status_code=422)

        # Validate session ID format if provided
        if query_data.sessionId:
            # Basic check for session ID format (in real implementation, this could be more strict)
            if len(query_data.sessionId) < 5 or len(query_data.sessionId) > 100:
                raise AppException("Session ID must be between 5 and 100 characters", status_code=422)

        # Validate user ID format if provided
        if query_data.userId:
            # Basic check for user ID format (in real implementation, this could be more strict)
            if len(query_data.userId) < 5 or len(query_data.userId) > 100:
                raise AppException("User ID must be between 5 and 100 characters", status_code=422)

        # Process the query through the RAG agent
        # In a real implementation, this would call the rag-backend service
        # For this implementation, we'll simulate the response
        response_text = await rag_agent.process_query(
            query=query_data.query,
            session_id=query_data.sessionId,
            user_id=query_data.userId
        )

        # Validate the response from the RAG agent
        if not response_text or "response" not in response_text:
            raise AppException("Invalid response from RAG agent", status_code=500)

        # Create a new chatbot interaction record
        interaction = await RAGChatbotService.create_interaction(
            RAGChatbotCreate(
                sessionId=query_data.sessionId,
                userId=query_data.userId,
                inputQuery=query_data.query,
                response=response_text.get("response", "I couldn't find a relevant answer to your question."),
                sourceChunks=response_text.get("sourceChunks", [])
            )
        )

        # Return the response
        return {
            "response": interaction.response,
            "sessionId": interaction.sessionId,
            "sourceChunks": interaction.sourceChunks,
            "timestamp": datetime.utcnow().isoformat()
        }

    except HTTPException:
        # Re-raise HTTP exceptions (like 429 for rate limiting)
        raise
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, "chatbot_query")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your query"
        )