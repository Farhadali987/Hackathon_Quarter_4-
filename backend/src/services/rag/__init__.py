from typing import List, Optional
from src.models.rag_chatbot import RAGChatbot, RAGChatbotCreate, RAGChatbotUpdate
from src.models.user_profile import User
from src.utils import NotFoundError
from datetime import datetime
import uuid


class RAGChatbotService:
    # In-memory storage for demonstration purposes
    # In a real application, this would connect to a database
    _storage: List[RAGChatbot] = []
    
    @classmethod
    async def create_interaction(cls, interaction_create: RAGChatbotCreate) -> RAGChatbot:
        """Create a new chatbot interaction"""
        new_interaction = RAGChatbot(
            id=str(uuid.uuid4()),
            sessionId=interaction_create.sessionId,
            userId=interaction_create.userId,
            inputQuery=interaction_create.inputQuery,
            response=interaction_create.response,
            sourceChunks=interaction_create.sourceChunks,
            createdAt=datetime.utcnow(),
            feedback=None,
            feedbackComment=None
        )
        
        cls._storage.append(new_interaction)
        return new_interaction
    
    @classmethod
    async def get_interaction_by_id(cls, interaction_id: str) -> Optional[RAGChatbot]:
        """Get a chatbot interaction by its ID"""
        for interaction in cls._storage:
            if interaction.id == interaction_id:
                return interaction
        return None
    
    @classmethod
    async def get_interactions_by_session(cls, session_id: str) -> List[RAGChatbot]:
        """Get all interactions for a given session"""
        return [i for i in cls._storage if i.sessionId == session_id]
    
    @classmethod
    async def get_interactions_by_user(cls, user_id: str) -> List[RAGChatbot]:
        """Get all interactions for a given user"""
        return [i for i in cls._storage if i.userId == user_id]
    
    @classmethod
    async def update_interaction_feedback(cls, interaction_id: str, feedback: int, feedback_comment: Optional[str] = None) -> Optional[RAGChatbot]:
        """Update feedback for a chatbot interaction"""
        for i, interaction in enumerate(cls._storage):
            if interaction.id == interaction_id:
                updated_interaction = interaction.copy()
                updated_interaction.feedback = feedback
                if feedback_comment is not None:
                    updated_interaction.feedbackComment = feedback_comment
                cls._storage[i] = updated_interaction
                return updated_interaction
        return None
    
    @classmethod
    async def get_recent_interactions(cls, limit: int = 10) -> List[RAGChatbot]:
        """Get the most recent chatbot interactions"""
        # Sort by creation date descending and return the top 'limit' interactions
        sorted_interactions = sorted(cls._storage, key=lambda x: x.createdAt, reverse=True)
        return sorted_interactions[:limit]
    
    @classmethod
    async def get_interactions_by_user_and_session(cls, user_id: str, session_id: str) -> List[RAGChatbot]:
        """Get interactions for a specific user and session"""
        return [i for i in cls._storage if i.userId == user_id and i.sessionId == session_id]


# Initialize with sample interactions if needed
async def initialize_sample_interactions():
    from src.models.rag_chatbot import RAGChatbotCreate
    
    sample_interactions = [
        RAGChatbotCreate(
            sessionId="session-1",
            userId="user-1",
            inputQuery="What is ROS 2?",
            response="ROS 2 is a flexible framework for developing robot applications...",
            sourceChunks=["chunk-1", "chunk-2"]
        )
    ]
    
    for interaction in sample_interactions:
        await RAGChatbotService.create_interaction(interaction)