import logging
from fastapi import HTTPException, status
from typing import Any, Dict
import traceback
from datetime import datetime


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("app.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)


class AppException(HTTPException):
    """Custom application exception"""
    def __init__(self, detail: str, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(status_code=status_code, detail=detail)
        logger.error(f"AppException: {detail}, Status: {status_code}")


class ValidationError(AppException):
    """Exception for validation errors"""
    def __init__(self, detail: str):
        super().__init__(detail=detail, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)


class NotFoundError(AppException):
    """Exception for not found errors"""
    def __init__(self, detail: str):
        super().__init__(detail=detail, status_code=status.HTTP_404_NOT_FOUND)


class UnauthorizedError(AppException):
    """Exception for unauthorized access"""
    def __init__(self, detail: str = "Unauthorized"):
        super().__init__(detail=detail, status_code=status.HTTP_401_UNAUTHORIZED)


def log_api_call(endpoint: str, method: str, user_id: str = None, success: bool = True):
    """Log API calls"""
    status = "SUCCESS" if success else "FAILED"
    user_info = f"User: {user_id}" if user_id else "User: Anonymous"
    logger.info(f"API Call: {method} {endpoint} - {user_info} - {status}")


def log_error(error: Exception, context: str = ""):
    """Log error with context and traceback"""
    logger.error(f"Error in {context}: {str(error)}")
    logger.error(f"Traceback: {traceback.format_exc()}")


def create_error_response(error: Exception, context: str = "") -> Dict[str, Any]:
    """Create standardized error response"""
    log_error(error, context)
    return {
        "error": str(error),
        "context": context,
        "timestamp": datetime.utcnow().isoformat(),
        "type": type(error).__name__
    }