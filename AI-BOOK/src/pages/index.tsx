import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import ModuleCard from '@site/src/components/ModuleCard';
import FloatingChatbotButton from '@site/src/components/FloatingChatbotButton'; // Import FloatingChatbotButton
import { Button } from '@mantine/core'; // Import Mantine Button

import styles from './index.module.css';

function HeroSection() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          PHYSICAL AI
        </Heading>
        <p className="hero__subtitle">Embodied Intelligence in the Physical World</p>
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
      <main>
        <HeroSection />

        <section className={styles.modulesSection}>
          <div className="container">
            <Heading as="h2" className="text--center">Modules</Heading>
            <div className={styles.modulesGrid}>
              <ModuleCard title="ROS 2 Nervous System" description="Develop robust robot control architectures." />
              <ModuleCard title="Digital Twin Simulation" description="Create and interact with virtual robot environments." />
              <ModuleCard title="NVIDIA Isaac AI" description="Leverage advanced AI for robotic perception and control." />
              <ModuleCard title="Vision-Language-Action" description="Integrate multimodal AI for intelligent robot behavior." />
              <ModuleCard title="Capstone Project" description="Apply all learned skills to a real-world robotics challenge." />
            </div>
          </div>
        </section>

        <section className={clsx('margin-top--lg', 'text--center', styles.startLearningSection)}>
          <div className="container">
            <Button variant="filled" color="neonCyan" size="xl" onClick={() => (window.location.href = '/docs/intro')}>
              Start Learning (Mantine)
            </Button>
          </div>
        </section>

        <section className={clsx('margin-top--lg', styles.keyResearchAreasSection)}>
          <div className="container">
            <Heading as="h2" className="text--center">Key Research Areas</Heading>
            <Accordion variant="filled" defaultValue="customization" radius="md">
              <Accordion.Item value="embodied-intelligence">
                <Accordion.Control>Embodied Intelligence</Accordion.Control>
                <Accordion.Panel>
                  <Text c="dimmed">
                    Exploring how physical bodies and environments influence intelligence and learning in robotic systems.
                    This involves understanding the interplay between perception, action, and physical interaction.
                  </Text>
                </Accordion.Panel>
              </Accordion.Item>

              <Accordion.Item value="humanoid-robotics">
                <Accordion.Control>Humanoid Robotics</Accordion.Control>
                <Accordion.Panel>
                  <Text c="dimmed">
                    Delving into the design, control, and autonomy of human-like robots, focusing on biomimicry,
                    dexterous manipulation, and natural human-robot interaction.
                  </Text>
                </Accordion.Panel>
              </Accordion.Item>

              <Accordion.Item value="bci-integration">
                <Accordion.Control>Brain-Computer Interface Integration</Accordion.Control>
                <Accordion.Panel>
                  <Text c="dimmed">
                    Investigating the fusion of biological and artificial intelligence, including direct neural control
                    of robotic prosthetics and advanced human-robot collaboration.
                  </Text>
                </Accordion.Panel>
              </Accordion.Item>
            </Accordion>
          </div>
        </section>
      </main>
      <FloatingChatbotButton />
    </Layout>
  );
}



