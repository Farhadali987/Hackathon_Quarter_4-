import time
from collections import defaultdict, deque
from typing import Dict, Deque


class RateLimiter:
    """
    A simple rate limiter that tracks requests by user ID or IP address
    """
    def __init__(self, max_requests: int = 10, window_seconds: int = 60):
        self.max_requests = max_requests
        self.window_seconds = window_seconds
        self.requests: Dict[str, Deque[float]] = defaultdict(deque)
    
    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed
        """
        current_time = time.time()
        # Remove requests that are outside the current window
        while (self.requests[identifier] and 
               current_time - self.requests[identifier][0] > self.window_seconds):
            self.requests[identifier].popleft()
        
        # Check if the number of requests is within the limit
        if len(self.requests[identifier]) < self.max_requests:
            self.requests[identifier].append(current_time)
            return True
        return False


# Global rate limiter instance
# In a real application, this would likely be configured with settings
rate_limiter = RateLimiter(max_requests=10, window_seconds=60)