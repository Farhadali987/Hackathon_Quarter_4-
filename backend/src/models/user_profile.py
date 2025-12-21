from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field, EmailStr


class UserProfileBase(BaseModel):
    email: EmailStr
    name: str
    preferredLanguage: str = "en"  # Language code (e.g., "en", "ur")
    learningLevel: str = "beginner"  # "beginner", "intermediate", "advanced"
    personalizationSettings: Dict[str, Any] = {}


class UserProfileCreate(UserProfileBase):
    password: str  # In a real app, this would be handled securely


class UserProfileUpdate(BaseModel):
    name: Optional[str] = None
    preferredLanguage: Optional[str] = Field(None, max_length=10)
    learningLevel: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced)$")
    personalizationSettings: Optional[Dict[str, Any]] = None


class UserProfile(UserProfileBase):
    id: str
    createdAt: datetime
    lastActiveAt: datetime
    progress: Dict[str, Any] = {}

    class Config:
        from_attributes = True