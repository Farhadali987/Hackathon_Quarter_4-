import time
from typing import Any, Optional, Dict
from collections import OrderedDict


class LRUCache:
    """
    A simple Least Recently Used (LRU) cache implementation
    """
    def __init__(self, capacity: int = 100, ttl: int = 300):  # 300 seconds = 5 minutes default TTL
        self.capacity = capacity
        self.ttl = ttl  # Time-to-live in seconds
        self.cache: OrderedDict[str, tuple] = OrderedDict()  # (value, timestamp)
    
    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache if it exists and hasn't expired
        """
        if key not in self.cache:
            return None
        
        value, timestamp = self.cache[key]
        
        # Check if the entry has expired
        if time.time() - timestamp > self.ttl:
            # Remove expired entry
            del self.cache[key]
            return None
        
        # Move to end (most recently used)
        self.cache.move_to_end(key)
        return value
    
    def put(self, key: str, value: Any) -> None:
        """
        Add or update a value in the cache
        """
        # Remove expired entries periodically (simple approach)
        current_time = time.time()
        expired_keys = [
            k for k, (_, timestamp) in self.cache.items()
            if current_time - timestamp > self.ttl
        ]
        for k in expired_keys:
            del self.cache[k]
        
        if key in self.cache:
            # Update existing key
            self.cache.move_to_end(key)
        elif len(self.cache) >= self.capacity:
            # Remove least recently used item (first item)
            self.cache.popitem(last=False)
        
        # Add new item with current timestamp
        self.cache[key] = (value, current_time)
    
    def invalidate(self, key: str) -> bool:
        """
        Remove a specific key from the cache
        """
        if key in self.cache:
            del self.cache[key]
            return True
        return False
    
    def clear(self) -> None:
        """
        Clear all entries from the cache
        """
        self.cache.clear()


# Global cache instance for textbook content
textbook_content_cache = LRUCache(capacity=50, ttl=600)  # 50 items, 10 minute TTL

# Global cache instance for user profiles
user_profile_cache = LRUCache(capacity=100, ttl=300)  # 100 items, 5 minute TTL

# Global cache instance for lab guidance
lab_guidance_cache = LRUCache(capacity=25, ttl=600)  # 25 items, 10 minute TTL