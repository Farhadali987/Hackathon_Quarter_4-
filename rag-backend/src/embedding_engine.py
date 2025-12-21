import asyncio
from typing import List, Tuple
from src.config import settings
from src.vector_store import vector_store
import numpy as np
from typing import Optional


class EmbeddingEngine:
    def __init__(self):
        # In a real implementation, we would initialize the embedding model here
        # For now, we'll simulate embeddings with random vectors
        pass
    
    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for the given texts.
        In a real implementation, this would call an embedding API like OpenAI's.
        """
        # Simulate embedding creation with random vectors of size 1536 (OpenAI's embedding size)
        embeddings = []
        for text in texts:
            # In a real implementation, we would call an embedding API
            # For simulation, we'll create a deterministic vector based on the text
            embedding = self._simulate_embedding(text)
            embeddings.append(embedding)
        return embeddings
    
    def _simulate_embedding(self, text: str) -> List[float]:
        """Simulate an embedding for a given text"""
        # This is a simple simulation - in reality, we'd use an ML model or API
        # Create a deterministic pseudo-embedding based on the text
        embedding = [0.0] * 1536
        for i, char in enumerate(text[:100]):  # Only use first 100 chars to keep deterministic
            embedding[i % 1536] += ord(char) / 255.0  # Normalize ASCII to 0-1 range
        
        # Normalize the embedding vector
        norm = np.linalg.norm(embedding)
        if norm > 0:
            embedding = [val / norm for val in embedding]
        
        return embedding
    
    async def embed_and_store_content(self, content_id: str, content: str, metadata: dict = None):
        """
        Split content into chunks, create embeddings, and store in vector database
        """
        if metadata is None:
            metadata = {}
        
        # Split content into chunks
        chunks = self._split_content_into_chunks(content)
        
        # Create embeddings for all chunks
        embeddings = self.create_embeddings(chunks)
        
        # Store each chunk with its embedding
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            chunk_metadata = metadata.copy()
            chunk_metadata["chunk_index"] = i
            chunk_metadata["total_chunks"] = len(chunks)
            
            await vector_store.add_embedding(
                content_id=content_id,
                embedding=embedding,
                text=chunk,
                metadata=chunk_metadata
            )
    
    def _split_content_into_chunks(self, content: str, chunk_size: int = None) -> List[str]:
        """
        Split content into chunks of specified size
        """
        if chunk_size is None:
            chunk_size = settings.CHUNK_SIZE
        
        chunks = []
        start = 0
        
        while start < len(content):
            end = start + chunk_size
            
            # Try to break at sentence boundary if possible
            if end < len(content):
                # Look for a sentence ending near the chunk boundary
                sentence_end = content.rfind('. ', start + chunk_size // 2, start + chunk_size)
                if sentence_end != -1 and sentence_end > start:
                    end = sentence_end + 2  # Include the period and space
            
            chunk = content[start:end].strip()
            if chunk:  # Only add non-empty chunks
                chunks.append(chunk)
            
            start = end
        
        return chunks
    
    async def search_relevant_content(self, query: str, limit: int = 5) -> List[dict]:
        """
        Search for content relevant to the query
        """
        query_embedding = self.create_embeddings([query])[0]
        results = await vector_store.search_similar(query_embedding, limit=limit)
        
        # Filter results based on similarity threshold
        filtered_results = [
            result for result in results 
            if result["score"] >= settings.SIMILARITY_THRESHOLD
        ]
        
        return filtered_results


# Global instance
embedding_engine = EmbeddingEngine()