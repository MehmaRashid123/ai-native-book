def to_urdu(text: str) -> str:
    """
    Goal: Secure Bonus Points for Urdu Translation.
    Function: to_urdu(text: str)
    Logic:
      - Create a placeholder function that accepts English text.
      - Return a string that mimics translation (e.g., prepend "[Urdu Translation]: ").
      - Add a comment that in production, this would call an LLM API to preserve technical keywords like "Lidar" and "SLAM".
    """
    # In production, this would call an LLM API to translate the text to Urdu
    # while preserving technical keywords like "Lidar", "SLAM", "ROS", etc.
    # For now, we'll just prepend a marker to indicate this is a mock translation

    return f"[Urdu Translation]: {text}"