import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <h1 className={styles.heroTitle}>
          Physical <span>AI</span> & <br />
          Humanoid Robotics
        </h1>
        <p className={styles.heroSubtitle}>
          {siteConfig.tagline}
        </p>
        <div className={styles.buttons}>
          <Link
            className={styles.neonBtn}
            to="/docs/intro/physical-ai">
            Start Learning ⚡
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="https://github.com/panaversity/ai-native-book">
            GitHub Repo
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main className={styles.sectionContainer}>
        <h2 className={styles.featureHeading}>Build the Future</h2>
        <p className={styles.featureText}>
          Master Embodied Intelligence with ROS 2, Isaac Sim, and VLA models through interactive simulations and real-world hardware labs.
        </p>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}