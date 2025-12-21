from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field


class TextbookContentBase(BaseModel):
    title: str = Field(..., min_length=1, max_length=200)
    content: str = Field(..., min_length=10, max_length=50000)
    contentType: str  # "chapter", "section", "lesson", "exercise"
    topic: str  # "ROS 2", "Gazebo", "NVIDIA Isaac", "VLA"
    level: str  # "beginner", "intermediate", "advanced"
    language: str = "en"  # Language code (e.g., "en", "ur")
    parentId: Optional[str] = None
    order: int = 0


class TextbookContentCreate(TextbookContentBase):
    pass


class TextbookContentUpdate(BaseModel):
    title: Optional[str] = Field(None, min_length=1, max_length=200)
    content: Optional[str] = Field(None, min_length=10, max_length=50000)
    contentType: Optional[str] = None
    topic: Optional[str] = None
    level: Optional[str] = None
    language: Optional[str] = None
    parentId: Optional[str] = None
    order: Optional[int] = None


class TextbookContent(TextbookContentBase):
    id: str
    createdAt: datetime
    updatedAt: datetime
    version: int = 1

    class Config:
        from_attributes = True