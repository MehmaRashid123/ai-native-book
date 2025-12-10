// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro-ros2',
        'module-1/python-agents',
        'module-1/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Simulation)',
      items: [
        'module-2/gazebo-physics',
        'module-2/unity-rendering',
        'module-2/sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/isaac-sim',
        'module-3/vslam',
        'module-3/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/whisper-voice',
        'module-4/llm-planning',
        'module-4/capstone',
      ],
    },
  ],
};

export default sidebars;