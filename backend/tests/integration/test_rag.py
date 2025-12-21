import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock
from src.main import app


@pytest.fixture
def client():
    with TestClient(app) as test_client:
        yield test_client


@pytest.mark.asyncio
@patch('src.api.chatbot.rag_agent', new_callable=AsyncMock)
def test_rag_chatbot_integration(mock_rag_agent, client):
    """Integration test for RAG chatbot functionality"""
    # Mock the RAG agent response
    mock_response = {
        "response": "ROS 2 is a flexible framework for developing robot applications.",
        "sourceChunks": ["chunk-1", "chunk-2"],
        "sessionId": "test-session-123"
    }
    mock_rag_agent.process_query.return_value = mock_response
    
    # Make request to the API
    payload = {
        "query": "What is ROS 2?",
        "sessionId": "test-session-123"
    }
    
    response = client.post("/api/chatbot/query", json=payload)
    
    # Assertions
    assert response.status_code == 200
    response_data = response.json()
    
    assert response_data["response"] == "ROS 2 is a flexible framework for developing robot applications."
    assert response_data["sessionId"] == "test-session-123"
    assert "timestamp" in response_data
    assert isinstance(response_data["sourceChunks"], list)


@patch('src.api.chatbot.rag_agent', new_callable=AsyncMock)
def test_rag_chatbot_with_user_context(mock_rag_agent, client):
    """Test RAG chatbot with user context"""
    # Mock the RAG agent response
    mock_response = {
        "response": "For beginners, I recommend starting with the basic ROS 2 tutorials.",
        "sourceChunks": ["chunk-3", "chunk-4"],
        "sessionId": "test-session-456"
    }
    mock_rag_agent.process_query.return_value = mock_response
    
    # Make request with user context
    payload = {
        "query": "How should I start learning ROS 2?",
        "sessionId": "test-session-456",
        "userId": "test-user-789"
    }
    
    response = client.post("/api/chatbot/query", json=payload)
    
    # Assertions
    assert response.status_code == 200
    response_data = response.json()
    
    assert "start learning" in response_data["response"].lower()
    assert response_data["sessionId"] == "test-session-456"
    assert isinstance(response_data["sourceChunks"], list)


def test_rag_chatbot_error_handling(client):
    """Test RAG chatbot error handling"""
    # Test with empty query
    payload = {
        "query": "",
        "sessionId": "test-session-789"
    }
    
    response = client.post("/api/chatbot/query", json=payload)
    
    # Should return an error for empty query
    assert response.status_code in [400, 422]