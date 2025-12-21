import os
from typing import List, Dict, Any
from pathlib import Path
import markdown
from bs4 import BeautifulSoup

class ContentIngestion:
    def __init__(self, content_dir: str = "frontend/docs"):
        self.content_dir = Path(content_dir)
    
    def read_textbook_content(self) -> List[Dict[str, Any]]:
        """Read all textbook content from markdown files"""
        content_list = []
        
        textbook_path = self.content_dir / "textbook"
        if not textbook_path.exists():
            return content_list
        
        for file_path in textbook_path.rglob("*.md"):
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                
                # Extract title from the first heading
                lines = content.split('\n')
                title = ""
                for line in lines:
                    if line.startswith('# '):
                        title = line[2:].strip()
                        break
                    elif line.startswith('---'):
                        continue  # Skip frontmatter if exists
                    elif line.strip():
                        title = line.strip()
                        break
                
                # Convert markdown to plain text for embedding
                html = markdown.markdown(content)
                text = BeautifulSoup(html, 'html.parser').get_text()
                
                content_list.append({
                    'title': title,
                    'content': content,
                    'text': text,
                    'file_path': str(file_path),
                    'topic': self._extract_topic_from_path(file_path)
                })
        
        return content_list
    
    def read_lab_content(self) -> List[Dict[str, Any]]:
        """Read all lab guidance content from markdown files"""
        content_list = []
        
        lab_path = self.content_dir / "lab-guides"
        if not lab_path.exists():
            return content_list
        
        for file_path in lab_path.rglob("*.md"):
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                
                # Extract title from the first heading
                lines = content.split('\n')
                title = ""
                for line in lines:
                    if line.startswith('# '):
                        title = line[2:].strip()
                        break
                    elif line.strip():
                        title = line.strip()
                        break
                
                # Convert markdown to plain text for embedding
                html = markdown.markdown(content)
                text = BeautifulSoup(html, 'html.parser').get_text()
                
                lab_type = "cloud" if "cloud" in str(file_path).lower() else "hardware"
                
                content_list.append({
                    'title': title,
                    'content': content,
                    'text': text,
                    'file_path': str(file_path),
                    'lab_type': lab_type
                })
        
        return content_list
    
    def _extract_topic_from_path(self, file_path: Path) -> str:
        """Extract topic from the file path"""
        # For example: chapter2-ros2.md -> ROS 2
        name = file_path.stem
        if name.startswith("chapter"):
            # Remove chapter number prefix
            topic = name.split('-', 1)[-1]
            # Convert to title case
            topic = topic.replace('-', ' ').title()
            # Special cases
            if "ros2" in topic.lower():
                return "ROS 2"
            elif "nvidia" in topic.lower():
                return "NVIDIA Isaac"
            elif "vla" in topic.lower():
                return "VLA"
            else:
                return topic
        return name