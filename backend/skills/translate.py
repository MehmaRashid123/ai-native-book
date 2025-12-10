def to_urdu(text: str) -> str:
    """
    Goal: Secure Bonus Points for Urdu Translation.
    Function: to_urdu(text: str)
    Logic:
      - Create a placeholder function that accepts English text.
      - Return a string that mimics translation (e.g., prepend "[Urdu Translation]: ").
      - Add a comment that in production, this would call an LLM API to preserve technical keywords like "Lidar" and "SLAM".
    """
    # Translation memory with specific mappings
    TRANSLATION_MEMORY = {
        "Welcome to the foundation of robotics - the Robot Operating System (ROS 2). In this module, you'll learn how robots communicate and coordinate their actions through a distributed network of nodes, topics, and services.": "Robotics ki bunyad mein khushamdeed - Robot Operating System (ROS 2). Is module mein aap seekhenge ke robots kis tarah nodes, topics aur services ke distributed network ke zariye aapas mein raabta aur amal karte hain.",
        "The fundamentals of ROS 2 architecture": "ROS 2 architecture ke bunyadi usool",
        "How to create and manage ROS 2 nodes": "ROS 2 nodes ko banana aur manage karna",
        "The publisher-subscriber communication pattern": "Publisher-subscriber raabtay ka tareeqa",
        "Using rclpy for Python-based robot control": "Python-based robot control ke liye rclpy ka istemal",
        "Robot Description Format (URDF) for robot modeling": "Robot modeling ke liye Robot Description Format (URDF)",
        "Basic Python programming knowledge": "Bunyadi Python programming ka ilm",
        "Understanding of object-oriented programming concepts": "Object-oriented programming concepts ki samajh",
        "What You'll Learn": "Aap Kya Seekhenge",
        "Prerequisites": "Zarooriat",
        "Getting Started": "Shuru Karein",
        "Physical AI & Humanoid Robotics": "Physical AI & Humanoid Robotics (جسمانی مصنوعی ذہانت اور ہیومنائزڈ روبوٹکس)",
        "Mastering the partnership between Humans, Agents, and Robots": "انسانوں، ایجنٹوں اور روبوٹس کے درمیان شراکت داری کو بہتر بنانا",
        "Module 1: ROS 2": "Module 1: ROS 2 (روبوٹ آپریٹنگ سسٹم 2)",
        "Module 2: Gazebo": "Module 2: Gazebo (گزیبو)",
        "Module 3: Isaac Sim": "Module 3: Isaac Sim (آئسیک سیم)",
        "Module 4: VLA": "Module 4: VLA (وژن لینگویج ایکشن)",
        "Learn the Robot Operating System fundamentals, Nodes, Topics, and rclpy.": "روبوٹ آپریٹنگ سسٹم کے بنیادیات، نوڈز، ٹوپکس اور rclpy سیکھیں۔",
        "Master physics simulation, collision detection, and sensor modeling.": "فزکس سیمولیشن، کولیژن ڈیٹیکشن اور سینسر ماڈلنگ ماسٹر کریں۔",
        "Explore NVIDIA's Isaac Sim, USD, VSLAM, and navigation systems.": "NVIDIA کے Isaac Sim، USD، VSLAM، اور نیویگیشن سسٹمز کو جانیں۔",
        "Implement Vision-Language-Action systems with Whisper and LLMs.": "Whisper اور LLMs کے ساتھ وژن-زبان-ایکشن سسٹمز نافذ کریں۔",
        "Modules": "Modules (ماڈیولز)",
        "Community": "Community (کمیونٹی)",
        "Stack Overflow": "Stack Overflow (اسٹیک اوور فلو)",
        "Discord": "Discord (ڈسکارڈ)",
        "Twitter": "Twitter (ٹویٹر)",
        "More": "More (مزید)",
        "Blog": "Blog ( بلاگ)",
        "GitHub": "GitHub (گیٹ ہب)",
        "Start Learning": "Seekhna Shuru Karein (سیکھنا شروع کریں)",
        "View Curriculum": "Curriculum Dekhein (نصاب دیکھیں)",
        "Introduction": "Taaruf (تعارف)",
        "Overview": "Jaiza (جائزہ)",
        "Robotics": "Robotics (روبوٹکس)",
        "Hardware": "Hardware (ہارڈ ویئر)",
        "System": "System (نظام)",
        "Agents": "Agents (ایجنٹس)",
        "Simulation": "Simulation (کمپیوٹر نقل)",
        "Digital Twin": "Digital Twin (ڈیجیٹل جڑواں)",
        "Sensors": "Sensors (سینسر)",
        "Vision": "Vision (بصارت)",
        "Language": "Language (زبان)",
        "Action": "Action (عمل)",
        "Getting Started": "Shuru Karein (شروع کریں)",
        "Prerequisites": "Zarooriat (ضروریات)",
        "Syllabus": "Nisaab (نصاب)",
        "Navigation": "Navigation (نیویگیشن)",
        "Artificial Intelligence": "Artificial Intelligence (مصنوعی ذہانت)",
        "Embodied Intelligence": "Embodied Intelligence (جسمانی ذہانت)",
        "Curriculum": "Curriculum (نصاب)",
        "Start Learning": "Start Learning (سیکھنا شروع کریں)",
        "View Curriculum": "View Curriculum (نصاب دیکھیں)",
        "Learn": "Sikhein (سیکھیں)",
        "The Robotic Nervous System (ROS 2)": "روبوٹک نروس سسٹم (ROS 2)",
        "The Digital Twin (Simulation)": "ڈیجیٹل ٹوئن (سمولیشن)",
        "The AI-Robot Brain (NVIDIA Isaac)": "AI-روبوٹ براہین (NVIDIA آئسیک)",
        "Vision-Language-Action (VLA)": "وژن-زبان-ایکشن (VLA)",
        "The Digital Twin (Gazebo Physics, Unity Rendering, Sensors)": "ڈیجیٹل ٹوئن (گزیبو فزکس، یونیٹی رینڈرنگ، سینسر)",
        "The AI-Robot Brain (NVIDIA Isaac Sim, VSLAM, Nav2)": "AI-روبوٹ براہین (NVIDIA آئسیک سیم، VSLAM، Nav2)",
        "Vision-Language-Action (OpenAI Whisper, LLM Cognitive Planning)": "وژن-زبان-ایکشن (OpenAI وِسپر، LLM کاگنیٹو پلاننگ)"
    }

    # Clean the input text (strip whitespace)
    cleaned_text = text.strip()

    # Check if text exists in TRANSLATION_MEMORY
    if cleaned_text in TRANSLATION_MEMORY:
        return TRANSLATION_MEMORY[cleaned_text]
    else:
        # Try to find partial matches for common patterns
        for eng_key, urdu_value in TRANSLATION_MEMORY.items():
            if cleaned_text.lower() in eng_key.lower() or eng_key.lower() in cleaned_text.lower():
                return urdu_value

        # Return a generic fallback if no match found
        return f"[AI] {text} (Urdu)"