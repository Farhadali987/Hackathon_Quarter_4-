from typing import List, Optional
from src.models.user_profile import UserProfile, UserProfileCreate, UserProfileUpdate
from src.utils import NotFoundError
from datetime import datetime
import uuid


class UserProfileService:
    # In-memory storage for demonstration purposes
    # In a real application, this would connect to a database
    _storage: List[UserProfile] = []
    
    @classmethod
    async def create_user(cls, user_create: UserProfileCreate) -> UserProfile:
        """Create a new user profile"""
        # Check if user with this email already exists
        for user in cls._storage:
            if user.email == user_create.email:
                raise ValueError(f"User with email {user_create.email} already exists")
        
        new_user = UserProfile(
            id=str(uuid.uuid4()),
            email=user_create.email,
            name=user_create.name,
            preferredLanguage=user_create.preferredLanguage,
            learningLevel=user_create.learningLevel,
            personalizationSettings=user_create.personalizationSettings,
            createdAt=datetime.utcnow(),
            lastActiveAt=datetime.utcnow(),
            progress={}
        )
        
        cls._storage.append(new_user)
        return new_user
    
    @classmethod
    async def get_user_by_id(cls, user_id: str) -> Optional[UserProfile]:
        """Get a user profile by ID"""
        for user in cls._storage:
            if user.id == user_id:
                return user
        return None
    
    @classmethod
    async def get_user_by_email(cls, email: str) -> Optional[UserProfile]:
        """Get a user profile by email"""
        for user in cls._storage:
            if user.email == email:
                return user
        return None
    
    @classmethod
    async def update_user(cls, user_id: str, user_update: UserProfileUpdate) -> Optional[UserProfile]:
        """Update a user profile"""
        for i, user in enumerate(cls._storage):
            if user.id == user_id:
                update_data = user_update.dict(exclude_unset=True)
                updated_user = user.copy(update=update_data)
                updated_user.lastActiveAt = datetime.utcnow()
                cls._storage[i] = updated_user
                return updated_user
        return None
    
    @classmethod
    async def update_user_progress(cls, user_id: str, progress_data: dict) -> Optional[UserProfile]:
        """Update user progress"""
        for i, user in enumerate(cls._storage):
            if user.id == user_id:
                updated_user = user.copy()
                updated_user.progress.update(progress_data)
                updated_user.lastActiveAt = datetime.utcnow()
                cls._storage[i] = updated_user
                return updated_user
        return None
    
    @classmethod
    async def get_all_users(cls) -> List[UserProfile]:
        """Get all user profiles"""
        return cls._storage.copy()
    
    @classmethod
    async def delete_user(cls, user_id: str) -> bool:
        """Delete a user profile"""
        for i, user in enumerate(cls._storage):
            if user.id == user_id:
                del cls._storage[i]
                return True
        return False
    
    @classmethod
    async def update_last_active(cls, user_id: str) -> Optional[UserProfile]:
        """Update the last active timestamp for a user"""
        for i, user in enumerate(cls._storage):
            if user.id == user_id:
                updated_user = user.copy()
                updated_user.lastActiveAt = datetime.utcnow()
                cls._storage[i] = updated_user
                return updated_user
        return None