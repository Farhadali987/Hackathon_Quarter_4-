from fastapi import APIRouter, Query, HTTPException, Depends
from typing import Optional, List
from datetime import datetime

from src.models.lab_guidance import LabGuidance
from src.services.content.lab_guidance_service import LabGuidanceService
from src.auth import get_current_user, User
from src.utils import log_api_call, log_error, AppException

router = APIRouter()


@router.get("/labs", summary="Retrieve lab guidance by various filters")
async def get_labs(
    type: Optional[str] = Query(None, description="Filter by lab type ('hardware', 'cloud', 'simulation')"),
    difficulty: Optional[str] = Query(None, description="Filter by difficulty level ('beginner', 'intermediate', 'advanced')"),
    topic: Optional[str] = Query(None, description="Filter by related topic"),
    current_user: User = Depends(get_current_user)
):
    """
    Retrieve lab guidance by various filters
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call("/api/labs", "GET", user_id)
        
        # Get labs with applied filters
        labs_list = await LabGuidanceService.get_labs_list(
            type=type,
            difficulty=difficulty,
            topic=topic
        )
        
        return {
            "labs": labs_list
        }
        
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, "get_labs")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving lab guidance"
        )


@router.get("/labs/{lab_id}", summary="Retrieve specific lab guidance by ID")
async def get_lab_by_id(
    lab_id: str,
    current_user: User = Depends(get_current_user)
):
    """
    Retrieve specific lab guidance by ID
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call(f"/api/labs/{lab_id}", "GET", user_id)
        
        # Get lab by ID
        lab = await LabGuidanceService.get_lab_by_id(lab_id)
        
        if lab is None:
            raise HTTPException(
                status_code=404,
                detail=f"Lab guidance with ID {lab_id} not found"
            )
        
        return lab
        
    except HTTPException:
        # Re-raise HTTP exceptions (like 404)
        raise
    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, f"get_lab_by_id({lab_id})")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving lab guidance"
        )


# Note: The LabGuidanceService doesn't exist yet, so I'll create a basic implementation
# In a real implementation, this would be properly implemented