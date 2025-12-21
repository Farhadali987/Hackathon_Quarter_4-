import openai
from typing import List
from .config import settings

class EmbeddingEngine:
    def __init__(self):
        if settings.openai_api_key:
            openai.api_key = settings.openai_api_key
        else:
            raise ValueError("OpenAI API key is required for embedding functionality")
    
    def create_embedding(self, text: str) -> List[float]:
        """Create embedding for a single text"""
        try:
            response = openai.Embedding.create(
                input=text,
                model="text-embedding-ada-002"
            )
            return response['data'][0]['embedding']
        except Exception as e:
            print(f"Error creating embedding: {e}")
            return []
    
    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Create embeddings for a list of texts"""
        embeddings = []
        for text in texts:
            embedding = self.create_embedding(text)
            if embedding:
                embeddings.append(embedding)
        return embeddings