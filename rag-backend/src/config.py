from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    QDRANT_HOST: str = "localhost"
    QDRANT_PORT: int = 6333
    OPENAI_API_KEY: str = ""  # In production, set this in environment
    EMBEDDING_MODEL: str = "text-embedding-ada-002"
    CHUNK_SIZE: int = 500  # Size of text chunks for embedding
    SIMILARITY_THRESHOLD: float = 0.7  # Minimum similarity score for retrieval
    
    class Config:
        env_file = ".env"


settings = Settings()