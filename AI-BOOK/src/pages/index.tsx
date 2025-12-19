import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

const modules = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn the core concepts of ROS 2: Nodes, Topics, Services, and Actions. Build your robot\'s communication backbone using rclpy and URDF.',
    link: '/docs/module-1-ros2/introduction',
  },
  {
    title: 'Module 2: The Digital Twin (Simulation)',
    description: 'Understand robot description formats (URDF/SDF) and simulate physics in Gazebo. Learn to add virtual sensors and test your robot in a safe environment.',
    link: '/docs/module-2-simulation/digital-twin-concept',
  },
  {
    title: 'Module 3: The AI Brain (NVIDIA Isaac)',
    description: 'Dive into NVIDIA Omniverse and Isaac Sim for advanced robotics simulation. Explore VSLAM, Nav2, and synthetic data generation for intelligent robots.',
    link: '/docs/module-3-isaac/intro-isaac-sim',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Connect Large Language Models (LLMs) like GPT-4 or Gemini to ROS 2 actions for human-like robot control. Implement voice commands with OpenAI Whisper and work towards your capstone project.',
    link: '/docs/module-4-vla/vla-theory',
  },
];


function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className={styles.heroTitle}>
          {siteConfig.title}
        </Heading>
        <p className={styles.heroSubtitle}>
          {siteConfig.tagline}
        </p>
        <Link className={styles.heroButton} to="/docs/module-1-ros2/introduction">
          Start Learning
        </Link>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <section className={styles.modulesSection}>
          <div className="container">
            <Heading as="h2" className="text--center">Modules</Heading>
            <div className={styles.modulesGrid}>
              {modules.map((module, idx) => (
                <Link to={module.link} key={idx} className={styles.moduleCardLink}>
                  <div className={clsx(styles.moduleCard, 'padding--lg')}>
                    <h3 style={{color: 'var(--neon-cyan)', fontWeight: 700, fontSize: '1.25rem'}}>{module.title}</h3>
                    <p style={{color: '#a0a0a0', marginTop: '0.5rem'}}>{module.description}</p>
                  </div>
                </Link>
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}