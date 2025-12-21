from typing import List, Dict, Any, Optional
from datetime import datetime
import uuid


class TranslationService:
    # In-memory storage for demonstration purposes
    # In a real application, this would connect to a database or translation API
    _translations: Dict[str, Dict[str, Any]] = {}
    _preserved_terms: List[str] = [
        "ROS 2", "Gazebo", "NVIDIA Isaac", "VLA", "SLAM", "PID Controller", 
        "Inverse Kinematics", "Forward Kinematics", "IMU", "LiDAR", "RGB-D", 
        "RANSAC", "ICP", "PCL", "OpenCV", "TensorFlow", "PyTorch", "CUDA", 
        "GPU", "CPU", "IoT", "API", "SDK", "TCP/IP", "HTTP", "HTTPS"
    ]
    
    @classmethod
    async def translate_content(cls, content_id: str, target_language: str, content: str) -> Dict[str, Any]:
        """
        Translate content to target language while preserving technical terminology
        """
        # In a real implementation, this would:
        # 1. Identify technical terms that should not be translated
        # 2. Use a translation service (with technical term preservation)
        # 3. Store the translated content
        
        # For this example, we'll simulate the process
        preserved_count = 0
        for term in cls._preserved_terms:
            if term in content:
                preserved_count += 1
        
        # Create a mock translation (in a real app, this would be actual translation)
        translated_content = f"[SIMULATED TRANSLATION] {content}"
        
        # Store the translation
        translation_record = {
            "id": str(uuid.uuid4()),
            "original_content_id": content_id,
            "target_language": target_language,
            "original_content": content,
            "translated_content": translated_content,
            "preserved_terms_count": preserved_count,
            "translated_terms_count": len(content.split()) - preserved_count,
            "status": "completed",
            "created_at": datetime.utcnow()
        }
        
        cls._translations[f"{content_id}_{target_language}"] = translation_record
        
        return translation_record
    
    @classmethod
    async def get_translation(cls, content_id: str, target_language: str) -> Optional[Dict[str, Any]]:
        """
        Get a previously translated content
        """
        return cls._translations.get(f"{content_id}_{target_language}")
    
    @classmethod
    async def get_preserved_terms(cls) -> List[str]:
        """
        Get the list of preserved technical terms
        """
        return cls._preserved_terms.copy()
    
    @classmethod
    async def add_preserved_term(cls, term: str) -> bool:
        """
        Add a new term to the preserved list
        """
        if term not in cls._preserved_terms:
            cls._preserved_terms.append(term)
            return True
        return False
    
    @classmethod
    async def remove_preserved_term(cls, term: str) -> bool:
        """
        Remove a term from the preserved list
        """
        if term in cls._preserved_terms:
            cls._preserved_terms.remove(term)
            return True
        return False
    
    @classmethod
    async def get_translated_content_for_user(cls, user_id: str, content_id: str, preferred_language: str) -> str:
        """
        Get appropriate content for user based on their language preference
        """
        if preferred_language == "en":
            # For English, return original content
            # In a real implementation, this would fetch from content storage
            return f"Original content for {content_id}"  # Placeholder
        else:
            # For other languages, try to get translated content
            translation = await cls.get_translation(content_id, preferred_language)
            if translation:
                return translation["translated_content"]
            else:
                # If no translation exists, return original content
                return f"Original content for {content_id}"  # Placeholder


# Initialize with sample preserved terms
async def initialize_preserved_terms():
    # The terms are already initialized in the class
    pass