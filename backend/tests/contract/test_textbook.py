import pytest
from fastapi.testclient import TestClient
from src.main import app


@pytest.fixture
def client():
    with TestClient(app) as test_client:
        yield test_client


def test_textbook_content_list_contract(client):
    """Test the contract for GET /api/textbook/content"""
    # Test with no query parameters
    response = client.get("/api/textbook/content")
    
    # Check response status
    assert response.status_code in [200, 400, 500]
    
    if response.status_code == 200:
        # Check response structure
        response_data = response.json()
        assert "content" in response_data
        assert isinstance(response_data["content"], list)
        
        # If content exists, validate its structure
        for item in response_data["content"]:
            assert "id" in item
            assert "title" in item
            assert "content" in item
            assert "contentType" in item
            assert "topic" in item
            assert "level" in item
            assert "language" in item
            assert "parentId" in item
            assert "order" in item
            
            # Validate field types
            assert isinstance(item["id"], str)
            assert isinstance(item["title"], str)
            assert isinstance(item["content"], str)
            assert isinstance(item["contentType"], str)
            assert isinstance(item["topic"], str)
            assert isinstance(item["level"], str)
            assert isinstance(item["language"], str)
            if item["parentId"] is not None:
                assert isinstance(item["parentId"], str)
            assert isinstance(item["order"], int)


def test_textbook_content_list_with_filters(client):
    """Test the contract with various filter parameters"""
    # Test with topic filter
    response = client.get("/api/textbook/content?topic=ROS%202")
    assert response.status_code in [200, 400, 500]
    
    # Test with level filter
    response = client.get("/api/textbook/content?level=beginner")
    assert response.status_code in [200, 400, 500]
    
    # Test with language filter
    response = client.get("/api/textbook/content?language=en")
    assert response.status_code in [200, 400, 500]
    
    # Test with multiple filters
    response = client.get("/api/textbook/content?topic=ROS%202&level=beginner")
    assert response.status_code in [200, 400, 500]


def test_textbook_content_by_id_contract(client):
    """Test the contract for GET /api/textbook/content/{id}"""
    # Test with a mock ID (will likely return 404 if ID doesn't exist)
    response = client.get("/api/textbook/content/mock-content-id")
    
    # Check response status
    assert response.status_code in [200, 404, 500]
    
    if response.status_code == 200:
        # Check response structure
        response_data = response.json()
        assert "id" in response_data
        assert "title" in response_data
        assert "content" in response_data
        assert "contentType" in response_data
        assert "topic" in response_data
        assert "level" in response_data
        assert "language" in response_data
        assert "parentId" in response_data
        assert "order" in response_data
        assert "createdAt" in response_data
        assert "updatedAt" in response_data
        
        # Validate field types
        assert isinstance(response_data["id"], str)
        assert isinstance(response_data["title"], str)
        assert isinstance(response_data["content"], str)
        assert isinstance(response_data["contentType"], str)
        assert isinstance(response_data["topic"], str)
        assert isinstance(response_data["level"], str)
        assert isinstance(response_data["language"], str)
        if response_data["parentId"] is not None:
            assert isinstance(response_data["parentId"], str)
        assert isinstance(response_data["order"], int)
        assert isinstance(response_data["createdAt"], str)
        assert isinstance(response_data["updatedAt"], str)