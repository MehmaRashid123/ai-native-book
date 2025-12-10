import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '3b3'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'f14'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '400'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '437'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '1eb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '00b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'a16'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'd49'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '78c'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '1cf'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/architecture',
                component: ComponentCreator('/docs/module-1-ros2/architecture', '4f2'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/python-agents',
                component: ComponentCreator('/docs/module-1-ros2/python-agents', '3e5'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/urdf',
                component: ComponentCreator('/docs/module-1-ros2/urdf', 'd3b'),
                exact: true
              },
              {
                path: '/docs/module-1/intro-ros2',
                component: ComponentCreator('/docs/module-1/intro-ros2', 'ec3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/python-agents',
                component: ComponentCreator('/docs/module-1/python-agents', '6ef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/urdf',
                component: ComponentCreator('/docs/module-1/urdf', '101'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/gazebo-physics',
                component: ComponentCreator('/docs/module-2/gazebo-physics', 'fd2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/sensors',
                component: ComponentCreator('/docs/module-2/sensors', 'd9d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/unity-rendering',
                component: ComponentCreator('/docs/module-2/unity-rendering', '0e2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3/isaac-sim',
                component: ComponentCreator('/docs/module-3/isaac-sim', '1b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3/nav2',
                component: ComponentCreator('/docs/module-3/nav2', '7b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3/vslam',
                component: ComponentCreator('/docs/module-3/vslam', '6ae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/capstone',
                component: ComponentCreator('/docs/module-4/capstone', 'c9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/llm-planning',
                component: ComponentCreator('/docs/module-4/llm-planning', 'ee9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4/whisper-voice',
                component: ComponentCreator('/docs/module-4/whisper-voice', 'cd4'),
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
    path: '/',
    component: ComponentCreator('/', '2b6'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
