from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field


class RAGChatbotBase(BaseModel):
    sessionId: str
    userId: Optional[str] = None
    inputQuery: str = Field(..., min_length=1, max_length=1000)
    response: str = Field(..., min_length=1, max_length=5000)
    sourceChunks: List[str] = []


class RAGChatbotCreate(RAGChatbotBase):
    pass


class RAGChatbotUpdate(BaseModel):
    response: Optional[str] = Field(None, min_length=1, max_length=5000)
    sourceChunks: Optional[List[str]] = None
    feedback: Optional[int] = Field(None, ge=1, le=5)
    feedbackComment: Optional[str] = None


class RAGChatbot(RAGChatbotBase):
    id: str
    createdAt: datetime
    feedback: Optional[int] = None  # User feedback rating (1-5) or thumbs up/down
    feedbackComment: Optional[str] = None

    class Config:
        from_attributes = True