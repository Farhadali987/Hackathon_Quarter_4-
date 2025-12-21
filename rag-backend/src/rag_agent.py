from typing import Dict, List, Optional
from src.embedding_engine import embedding_engine
from src.vector_store import vector_store
from src.config import settings


class RAGAgent:
    def __init__(self):
        self.embedding_engine = embedding_engine
        self.vector_store = vector_store
        self.max_context_length = 2000  # Maximum characters for context
    
    async def process_query(self, query: str, session_id: str, user_id: Optional[str] = None) -> Dict:
        """
        Process a user query using RAG (Retrieval Augmented Generation)
        """
        try:
            # 1. Find relevant content based on the query
            relevant_chunks = await self.embedding_engine.search_relevant_content(query)
            
            if not relevant_chunks:
                return {
                    "response": "I couldn't find any relevant content to answer your question. Please try rephrasing your query.",
                    "sourceChunks": []
                }
            
            # 2. Prepare context from retrieved chunks
            context = self._prepare_context(relevant_chunks)
            
            # 3. Generate response based on context and query
            # In a real implementation, this would call an LLM API
            # For this implementation, we'll create a simulated response
            response = await self._generate_response(query, context)
            
            # 4. Return the response with source information
            return {
                "response": response,
                "sourceChunks": [chunk["content_id"] for chunk in relevant_chunks]
            }
            
        except Exception as e:
            # In a real implementation, we would have more sophisticated error handling
            return {
                "response": "An error occurred while processing your query. Please try again later.",
                "sourceChunks": []
            }
    
    def _prepare_context(self, chunks: List[Dict]) -> str:
        """
        Prepare context from retrieved chunks
        """
        # Combine the text from all chunks to form the context
        context_parts = []
        total_length = 0
        
        for chunk in chunks:
            chunk_text = chunk.get("text", "")
            if total_length + len(chunk_text) > self.max_context_length:
                # Stop adding chunks if we exceed the max context length
                break
            
            context_parts.append(chunk_text)
            total_length += len(chunk_text)
        
        return "\n\n".join(context_parts)
    
    async def _generate_response(self, query: str, context: str) -> str:
        """
        Generate a response based on the query and context
        In a real implementation, this would call an LLM API
        """
        # This is a simplified implementation
        # In a real system, we would use an LLM to generate the response
        
        # For demonstration purposes, we'll create a response that incorporates the context
        if "what is" in query.lower() or "what's" in query.lower():
            # Simple response for "what is" questions
            if len(context) > 0:
                # Extract the first sentence from context that might answer the question
                sentences = context.split('. ')
                for sentence in sentences:
                    if query.lower().replace("what is ", "").replace("what's ", "") in sentence.lower():
                        return sentence.strip() + "."
                # If no direct match, return first sentence of context
                return sentences[0].strip() + "." if sentences else "I found relevant information but couldn't extract a direct answer."
            else:
                return "I couldn't find any information about that topic in the textbook."
        elif "how" in query.lower():
            # Simple response for "how" questions
            if len(context) > 0:
                return f"Based on the textbook content, here's how: {context[:200]}..."
            else:
                return "I couldn't find any procedural information about that topic in the textbook."
        else:
            # General response
            if len(context) > 0:
                return f"According to the textbook: {context[:300]}..."
            else:
                return "I couldn't find any relevant information in the textbook."
    
    async def add_content(self, content_id: str, content: str, metadata: dict = None):
        """
        Add content to the RAG system for future queries
        """
        await self.embedding_engine.embed_and_store_content(content_id, content, metadata)
    
    async def update_content(self, content_id: str, new_content: str, metadata: dict = None):
        """
        Update existing content in the RAG system
        """
        # First, remove the old content
        await self.vector_store.delete_content(content_id)
        
        # Then add the new content
        await self.add_content(content_id, new_content, metadata)
    
    async def delete_content(self, content_id: str):
        """
        Remove content from the RAG system
        """
        await self.vector_store.delete_content(content_id)