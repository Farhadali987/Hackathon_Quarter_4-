from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime
import uuid

class UserProfile(BaseModel):
    id: str = str(uuid.uuid4())
    username: str
    email: str
    preferred_language: str = "en"
    personalization_settings: Optional[Dict[str, Any]] = {}
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    learning_progress: Optional[Dict[str, Any]] = {}