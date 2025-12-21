import os
from typing import Optional
from pydantic import BaseSettings

class Settings(BaseSettings):
    # Database settings
    database_url: str = os.getenv("DATABASE_URL", "")
    
    # Qdrant settings
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    
    # Authentication settings
    auth_secret: str = os.getenv("BETTER_AUTH_SECRET", "dev-secret-key-change-in-production")
    auth_algorithm: str = "HS256"
    auth_access_token_expire_minutes: int = 30
    
    # Application settings
    app_name: str = "AI Textbook for Physical AI & Humanoid Robotics"
    app_version: str = "1.0.0"
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"
    
    # OpenAI settings
    openai_api_key: Optional[str] = os.getenv("OPENAI_API_KEY")
    
    class Config:
        env_file = ".env"

settings = Settings()