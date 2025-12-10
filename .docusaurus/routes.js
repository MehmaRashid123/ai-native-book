import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ur/docs',
    component: ComponentCreator('/ur/docs', '77d'),
    routes: [
      {
        path: '/ur/docs',
        component: ComponentCreator('/ur/docs', 'ee9'),
        routes: [
          {
            path: '/ur/docs',
            component: ComponentCreator('/ur/docs', 'ca5'),
            routes: [
              {
                path: '/ur/docs/intro',
                component: ComponentCreator('/ur/docs/intro', '7af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-1-ros2/architecture',
                component: ComponentCreator('/ur/docs/module-1-ros2/architecture', 'a72'),
                exact: true
              },
              {
                path: '/ur/docs/module-1-ros2/python-agents',
                component: ComponentCreator('/ur/docs/module-1-ros2/python-agents', '475'),
                exact: true
              },
              {
                path: '/ur/docs/module-1-ros2/urdf',
                component: ComponentCreator('/ur/docs/module-1-ros2/urdf', '861'),
                exact: true
              },
              {
                path: '/ur/docs/module-1/intro-ros2',
                component: ComponentCreator('/ur/docs/module-1/intro-ros2', 'fa9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-1/python-agents',
                component: ComponentCreator('/ur/docs/module-1/python-agents', 'd12'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-1/urdf',
                component: ComponentCreator('/ur/docs/module-1/urdf', '276'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-2/gazebo-physics',
                component: ComponentCreator('/ur/docs/module-2/gazebo-physics', '418'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-2/sensors',
                component: ComponentCreator('/ur/docs/module-2/sensors', '747'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-2/unity-rendering',
                component: ComponentCreator('/ur/docs/module-2/unity-rendering', '0da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-3/isaac-sim',
                component: ComponentCreator('/ur/docs/module-3/isaac-sim', '479'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-3/nav2',
                component: ComponentCreator('/ur/docs/module-3/nav2', 'e89'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-3/vslam',
                component: ComponentCreator('/ur/docs/module-3/vslam', '130'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-4/capstone',
                component: ComponentCreator('/ur/docs/module-4/capstone', '074'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-4/llm-planning',
                component: ComponentCreator('/ur/docs/module-4/llm-planning', 'bd8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/module-4/whisper-voice',
                component: ComponentCreator('/ur/docs/module-4/whisper-voice', '559'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ur/',
    component: ComponentCreator('/ur/', '8d7'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
