def search_knowledge_base(query: str) -> str:
    """
    Goal: Base requirement for the Chatbot.
    Function: search_knowledge_base(query: str)
    Logic:
      - Create a placeholder function that simulates searching a Vector Database (Qdrant).
      - If query contains "ROS" or "Node", return a mock string about "Module 1: ROS 2 Architecture."
      - If query contains "Isaac" or "Sim", return a mock string about "Module 3: NVIDIA Isaac Sim Requirements."
    """
    query_lower = query.lower()

    if "ros" in query_lower or "node" in query_lower:
        return "Module 1: ROS 2 Architecture. ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of libraries and tools that help developers create robot applications. A ROS 2 node is an executable that uses ROS to communicate with other nodes. Nodes are organized into packages, which contain source code, configuration files, and other resources needed to run the node."

    elif "isaac" in query_lower or "sim" in query_lower:
        return "Module 3: NVIDIA Isaac Sim Requirements. NVIDIA Isaac Sim is a robotics simulator that provides a rich set of features for simulating robots and their environments. It requires an RTX GPU with at least 12GB VRAM for optimal performance. The simulator supports USD (Universal Scene Description) for scene representation and provides physics simulation, sensor simulation, and rendering capabilities."

    else:
        # Default response for other queries
        return "I found relevant information in the textbook. Please refer to the appropriate module for detailed information on this topic."