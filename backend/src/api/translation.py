from fastapi import APIRouter, HTTPException, Depends
from typing import List
from datetime import datetime

from src.auth import get_current_user, User
from src.utils import log_api_call, log_error, AppException

router = APIRouter()


@router.get("/translation/available", summary="Retrieve list of available languages for translation")
async def get_available_languages(
    current_user: User = Depends(get_current_user)
):
    """
    Retrieve list of available languages for translation
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call("/api/translation/available", "GET", user_id)

        # Define available languages
        # In a real implementation, this would come from a database or configuration
        available_languages = [
            {
                "code": "en",
                "name": "English",
                "isSupported": True
            },
            {
                "code": "ur",
                "name": "Urdu",
                "isSupported": True
            },
            {
                "code": "es",
                "name": "Spanish",
                "isSupported": False
            },
            {
                "code": "fr",
                "name": "French",
                "isSupported": False
            }
        ]

        return {
            "languages": available_languages
        }

    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, "get_available_languages")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving available languages"
        )


@router.post("/translation/content", summary="Translate textbook content to target language")
async def translate_content(
    content_id: str,
    target_language: str,
    current_user: User = Depends(get_current_user)
):
    """
    Translate textbook content to the target language while preserving technical terminology
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call(f"/api/translation/content/{content_id}", "POST", user_id)

        # Validate target language
        if target_language not in ["en", "ur"]:
            raise AppException(f"Language {target_language} is not supported for translation", status_code=400)

        # In a real implementation, this would:
        # 1. Retrieve the content by content_id
        # 2. Identify technical terms that should not be translated
        # 3. Use a translation service (with technical term preservation)
        # 4. Store the translated content
        # 5. Return the translated content

        # For this example, we'll return a mock response
        translated_content = {
            "original_content_id": content_id,
            "target_language": target_language,
            "translated_content": f"Mock translation of content {content_id} to {target_language}",
            "status": "completed",
            "preserved_terms_count": 0,
            "translated_terms_count": 0
        }

        return translated_content

    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, f"translate_content({content_id}, {target_language})")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while translating content"
        )


@router.get("/translation/terms", summary="Retrieve preserved technical terminology")
async def get_preserved_terms(
    current_user: User = Depends(get_current_user)
):
    """
    Retrieve list of technical terms that are preserved during translation
    """
    try:
        # Log the API call
        user_id = current_user.id if current_user else None
        log_api_call("/api/translation/terms", "GET", user_id)

        # In a real implementation, this would fetch from a database
        # For this example, we'll return some common technical terms
        preserved_terms = [
            "ROS 2", "Gazebo", "NVIDIA Isaac", "VLA", "SLAM", "PID Controller",
            "Inverse Kinematics", "Forward Kinematics", "IMU", "LiDAR", "RGB-D",
            "RANSAC", "ICP", "PCL", "OpenCV", "TensorFlow", "PyTorch", "CUDA",
            "GPU", "CPU", "IoT", "API", "SDK", "TCP/IP", "HTTP", "HTTPS"
        ]

        return {
            "preserved_terms": preserved_terms
        }

    except AppException:
        # Re-raise application-specific exceptions
        raise
    except Exception as e:
        # Log the error and return a generic error response
        log_error(e, "get_preserved_terms")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while retrieving preserved terms"
        )