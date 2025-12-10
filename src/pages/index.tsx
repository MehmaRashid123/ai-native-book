import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">
          Mastering the partnership between Humans, Agents, and Robots
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
          <Link
            className="button button--outline button--secondary button--lg margin-left--md"
            to="/docs/intro">
            View Curriculum
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ title, description, moduleNumber, icon }) {
  const moduleClass = `module-${moduleNumber}`;
  return (
    <div className={`col col--3 ${styles.featureCard} ${moduleClass}`}>
      <div className="card">
        <div className="card__body">
          <h3>{title}</h3>
          <p>{description}</p>
          <div className={styles.icon}>{icon}</div>
        </div>
      </div>
    </div>
  );
}

function Features() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <FeatureCard
            title="Module 1: ROS 2"
            description="Learn the Robot Operating System fundamentals, Nodes, Topics, and rclpy."
            moduleNumber={1}
            icon="🤖"
          />
          <FeatureCard
            title="Module 2: Gazebo"
            description="Master physics simulation, collision detection, and sensor modeling."
            moduleNumber={2}
            icon="🏭"
          />
          <FeatureCard
            title="Module 3: Isaac Sim"
            description="Explore NVIDIA's Isaac Sim, USD, VSLAM, and navigation systems."
            moduleNumber={3}
            icon="🎮"
          />
          <FeatureCard
            title="Module 4: VLA"
            description="Implement Vision-Language-Action systems with Whisper and LLMs."
            moduleNumber={4}
            icon="🧠"
          />
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        <Features />
      </main>
    </Layout>
  );
}