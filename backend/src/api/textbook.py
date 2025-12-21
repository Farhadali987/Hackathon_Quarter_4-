from fastapi import APIRouter, Query, HTTPException
from typing import Optional, List
from datetime import datetime

from src.models.textbook_content import TextbookContent
from src.services.content import TextbookContentService
from src.auth import get_current_user, User
from src.utils import log_api_call, log_error, AppException
from src.utils.caching import textbook_content_cache

router = APIRouter()


@router.get("/textbook/content", summary="Retrieve textbook content by various filters")
async def get_textbook_content(
    topic: Optional[str] = Query(None, description="Filter by topic (e.g., 'ROS 2', 'Gazebo')"),
    level: Optional[str] = Query(None, description="Filter by difficulty level ('beginner', 'intermediate', 'advanced')"),
    language: Optional[str] = Query(None, description="Filter by language ('en', 'ur')"),
    content_type: Optional[str] = Query(None, description="Filter by content type ('chapter', 'section', 'lesson')"),
    parent_id: Optional[str] = Query(None, description="Get content that is a child of the specified parent"),
    current_user: User = Depends(get_current_user)
):
    """
    Retrieve textbook content by various filters
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call("/api/textbook/content", "GET", user_id)

        # Create a cache key based on the parameters
        cache_key = f"textbook_content:{topic}:{level}:{language}:{content_type}:{parent_id}"

        # Check if result is in cache
        cached_result = textbook_content_cache.get(cache_key)
        if cached_result is not None:
            print(f"Cache hit for key: {cache_key}")
            return cached_result

        # Log the filter parameters for debugging
        filters = {
            "topic": topic,
            "level": level,
            "language": language,
            "content_type": content_type,
            "parent_id": parent_id
        }
        # Only log non-None filters
        active_filters = {k: v for k, v in filters.items() if v is not None}
        if active_filters:
            print(f"Textbook content query with filters: {active_filters}")

        # Get content with applied filters
        content_list = await TextbookContentService.get_content_list(
            topic=topic,
            level=level,
            language=language,
            content_type=content_type,
            parent_id=parent_id
        )

        # Log successful retrieval
        print(f"Retrieved {len(content_list)} content items for user {user_id}")

        # Prepare result
        result = {
            "content": content_list
        }

        # Cache the result
        textbook_content_cache.put(cache_key, result)

        return result

    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, "get_textbook_content")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving textbook content"
        )


@router.get("/textbook/content/{content_id}", summary="Retrieve specific textbook content by ID")
async def get_textbook_content_by_id(
    content_id: str,
    current_user: User = Depends(get_current_user)
):
    """
    Retrieve specific textbook content by ID
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call(f"/api/textbook/content/{content_id}", "GET", user_id)
        
        # Get content by ID
        content = await TextbookContentService.get_content_by_id(content_id)
        
        if content is None:
            raise HTTPException(
                status_code=404,
                detail=f"Textbook content with ID {content_id} not found"
            )
        
        return content
        
    except HTTPException:
        # Re-raise HTTP exceptions (like 404)
        raise
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, f"get_textbook_content_by_id({content_id})")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving textbook content"
        )


@router.post("/textbook/content", summary="Create new textbook content")
async def create_textbook_content(
    content: TextbookContent,
    current_user: User = Depends(get_current_user)
):
    """
    Create new textbook content
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call("/api/textbook/content", "POST", user_id)
        
        # In a real implementation, we would validate that the user has permission to create content
        # For this implementation, we'll assume the user has permission
        
        # Create the content
        created_content = await TextbookContentService.create_content(content)
        
        return created_content
        
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, "create_textbook_content")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while creating textbook content"
        )


@router.put("/textbook/content/{content_id}", summary="Update textbook content by ID")
async def update_textbook_content(
    content_id: str,
    content_update: TextbookContent,
    current_user: User = Depends(get_current_user)
):
    """
    Update textbook content by ID
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call(f"/api/textbook/content/{content_id}", "PUT", user_id)
        
        # In a real implementation, we would validate that the user has permission to update content
        # For this implementation, we'll assume the user has permission
        
        # Update the content
        updated_content = await TextbookContentService.update_content(content_id, content_update)
        
        if updated_content is None:
            raise HTTPException(
                status_code=404,
                detail=f"Textbook content with ID {content_id} not found"
            )
        
        return updated_content
        
    except HTTPException:
        # Re-raise HTTP exceptions (like 404)
        raise
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, f"update_textbook_content({content_id})")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while updating textbook content"
        )


@router.delete("/textbook/content/{content_id}", summary="Delete textbook content by ID")
async def delete_textbook_content(
    content_id: str,
    current_user: User = Depends(get_current_user)
):
    """
    Delete textbook content by ID
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call(f"/api/textbook/content/{content_id}", "DELETE", user_id)
        
        # In a real implementation, we would validate that the user has permission to delete content
        # For this implementation, we'll assume the user has permission
        
        # Delete the content
        success = await TextbookContentService.delete_content(content_id)
        
        if not success:
            raise HTTPException(
                status_code=404,
                detail=f"Textbook content with ID {content_id} not found"
            )
        
        return {"message": f"Textbook content with ID {content_id} deleted successfully"}
        
    except HTTPException:
        # Re-raise HTTP exceptions (like 404)
        raise
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, f"delete_textbook_content({content_id})")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while deleting textbook content"
        )