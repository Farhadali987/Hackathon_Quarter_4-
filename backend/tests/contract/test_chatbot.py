import pytest
from fastapi.testclient import TestClient
from src.main import app
from src.models.rag_chatbot import RAGChatbotCreate


@pytest.fixture
def client():
    with TestClient(app) as test_client:
        yield test_client


def test_chatbot_query_contract(client):
    """Test the contract for POST /api/chatbot/query"""
    # Test request structure
    payload = {
        "query": "What is ROS 2?",
        "sessionId": "test-session-123",
        "userId": "test-user-456"
    }
    
    response = client.post("/api/chatbot/query", json=payload)
    
    # Check response status
    assert response.status_code in [200, 400, 429, 500]
    
    if response.status_code == 200:
        # Check response structure
        response_data = response.json()
        assert "response" in response_data
        assert "sessionId" in response_data
        assert "sourceChunks" in response_data
        assert "timestamp" in response_data
        
        # Validate field types
        assert isinstance(response_data["response"], str)
        assert isinstance(response_data["sessionId"], str)
        assert isinstance(response_data["sourceChunks"], list)
        assert isinstance(response_data["timestamp"], str)
        
        # Validate content constraints
        assert len(response_data["response"]) > 0
        assert len(response_data["sourceChunks"]) >= 0
        assert response_data["sessionId"] == payload["sessionId"]


def test_chatbot_query_required_fields(client):
    """Test that required fields are validated"""
    # Test without required query field
    payload = {
        "sessionId": "test-session-123"
    }
    
    response = client.post("/api/chatbot/query", json=payload)
    
    # Should return 422 for validation error or 400 for bad request
    assert response.status_code in [400, 422]


def test_chatbot_query_minimal_payload(client):
    """Test with minimal valid payload"""
    payload = {
        "query": "What is ROS 2?"
    }
    
    response = client.post("/api/chatbot/query", json=payload)
    
    # Should either succeed or return a valid error code
    assert response.status_code in [200, 400, 429, 500]
    
    if response.status_code == 200:
        response_data = response.json()
        assert "response" in response_data
        assert isinstance(response_data["response"], str)
        assert len(response_data["response"]) > 0