from typing import List, Optional
from src.models.lab_guidance import LabGuidance, LabGuidanceCreate, LabGuidanceUpdate
from src.utils import NotFoundError
from datetime import datetime
import uuid


class LabGuidanceService:
    # In-memory storage for demonstration purposes
    # In a real application, this would connect to a database
    _storage: List[LabGuidance] = []
    
    @classmethod
    async def create_lab(cls, lab_create: LabGuidanceCreate) -> LabGuidance:
        """Create a new lab guidance item"""
        new_lab = LabGuidance(
            id=str(uuid.uuid4()),
            title=lab_create.title,
            description=lab_create.description,
            type=lab_create.type,
            difficulty=lab_create.difficulty,
            duration=lab_create.duration,
            instructions=lab_create.instructions,
            requirements=lab_create.requirements,
            learningObjectives=lab_create.learningObjectives,
            createdAt=datetime.utcnow(),
            updatedAt=datetime.utcnow()
        )
        
        cls._storage.append(new_lab)
        return new_lab
    
    @classmethod
    async def get_lab_by_id(cls, lab_id: str) -> Optional[LabGuidance]:
        """Get a lab guidance item by its ID"""
        for lab in cls._storage:
            if lab.id == lab_id:
                return lab
        return None
    
    @classmethod
    async def get_labs_list(
        cls, 
        type: Optional[str] = None, 
        difficulty: Optional[str] = None, 
        topic: Optional[str] = None
    ) -> List[LabGuidance]:
        """Get a list of lab guidance items with optional filters"""
        results = cls._storage
        
        if type:
            results = [c for c in results if c.type.lower() == type.lower()]
        if difficulty:
            results = [c for c in results if c.difficulty.lower() == difficulty.lower()]
        # Topic filtering would require searching in description, title, or requirements
        if topic:
            results = [c for c in results 
                      if topic.lower() in c.description.lower() 
                      or topic.lower() in c.title.lower()
                      or any(topic.lower() in req.lower() for req in c.requirements)]
        
        return results
    
    @classmethod
    async def update_lab(cls, lab_id: str, lab_update: LabGuidanceUpdate) -> Optional[LabGuidance]:
        """Update a lab guidance item"""
        from datetime import datetime
        
        for i, lab in enumerate(cls._storage):
            if lab.id == lab_id:
                update_data = lab_update.dict(exclude_unset=True)
                updated_lab = lab.copy(update=update_data)
                updated_lab.updatedAt = datetime.utcnow()
                cls._storage[i] = updated_lab
                return updated_lab
        return None
    
    @classmethod
    async def delete_lab(cls, lab_id: str) -> bool:
        """Delete a lab guidance item"""
        for i, lab in enumerate(cls._storage):
            if lab.id == lab_id:
                del cls._storage[i]
                return True
        return False


# Initialize with some sample labs
async def initialize_sample_labs():
    from src.models.lab_guidance import LabGuidanceCreate
    
    sample_labs = [
        LabGuidanceCreate(
            title="Getting Started with ROS 2",
            description="Learn the basics of ROS 2 by setting up a simple publisher and subscriber",
            type="simulation",
            difficulty="beginner",
            duration=60,
            instructions="1. Install ROS 2 Humble Hawksbill\n2. Create a new workspace\n3. Create a publisher node\n4. Create a subscriber node\n5. Run the nodes and observe communication",
            requirements=["ROS 2 Humble", "Linux/Ubuntu 20.04+"],
            learningObjectives=["Understand ROS 2 nodes", "Learn about topics and messages", "Practice basic ROS 2 commands"]
        ),
        LabGuidanceCreate(
            title="Controlling a Simulated Robot Arm",
            description="Control a simulated robot arm to pick and place objects",
            type="simulation",
            difficulty="intermediate",
            duration=90,
            instructions="1. Launch the robot arm simulation\n2. Plan a trajectory to approach an object\n3. Execute the grasp\n4. Move the object to a target location\n5. Release the object",
            requirements=["ROS 2", "MoveIt", "Gazebo"],
            learningObjectives=["Understand robot kinematics", "Learn trajectory planning", "Practice manipulation skills"]
        )
    ]
    
    for lab in sample_labs:
        await LabGuidanceService.create_lab(lab)