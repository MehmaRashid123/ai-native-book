import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import { Button, Text, Card } from '@mantine/core'; // Import Mantine Components

import FloatingChatbotButton from '@site/src/components/FloatingChatbotButton'; // Import FloatingChatbotButton
import ModuleCard from '@site/src/components/ModuleCard'; // ModuleCard is removed, but for now Mantine Card is here
import styles from './index.module.css';

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
        <Button variant="filled" color="neonCyan" size="xl" onClick={() => (window.location.href = '/docs/intro')} mt="xl">
          Start Learning (Mantine)
        </Button>
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
              <Card shadow="sm" padding="lg" radius="md" withBorder className={styles.moduleCard}>
                <Text color="neonCyan" fw={700} fz="lg">ROS 2 Nervous System</Text>
                <Text c="dimmed" mt="xs">Develop robust robot control architectures.</Text>
              </Card>
              <Card shadow="sm" padding="lg" radius="md" withBorder className={styles.moduleCard}>
                <Text color="neonCyan" fw={700} fz="lg">Digital Twin Simulation</Text>
                <Text c="dimmed" mt="xs">Create and interact with virtual robot environments.</Text>
              </Card>
              <Card shadow="sm" padding="lg" radius="md" withBorder className={styles.moduleCard}>
                <Text color="neonCyan" fw={700} fz="lg">NVIDIA Isaac AI</Text>
                <Text c="dimmed" mt="xs">Leverage advanced AI for robotic perception and control.</Text>
              </Card>
              <Card shadow="sm" padding="lg" radius="md" withBorder className={styles.moduleCard}>
                <Text color="neonCyan" fw={700} fz="700" fz="lg">Vision-Language-Action</Text>
                <Text c="dimmed" mt="xs">Integrate multimodal AI for intelligent robot behavior.</Text>
              </Card>
              <Card shadow="sm" padding="lg" radius="md" withBorder className={styles.moduleCard}>
                <Text color="neonCyan" fw={700} fz="lg">Capstone Project</Text>
                <Text c="dimmed" mt="xs">Apply all learned skills to a real-world robotics challenge.</Text>
              </Card>
            </div>
          </div>
        </section>
      </main>
      <FloatingChatbotButton />
    </Layout>
  );
}