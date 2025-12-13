import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import ModuleCard from '@site/src/components/ModuleCard';
import SkillCard from '@site/src/components/SkillCard';
import AgentCard from '@site/src/components/AgentCard';
import FloatingChatbotButton from '@site/src/components/FloatingChatbotButton'; // Import FloatingChatbotButton

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

        <section className={styles.skillsSection}>
          <div className="container">
            <Heading as="h2" className="text--center">AI-Native Skills</Heading>
            <div className={styles.skillsGrid}>
              <SkillCard title="Robotics Systems Thinking" />
              <SkillCard title="Physical AI Design" />
              <SkillCard title="Simulation & Digital Twins" />
              <SkillCard title="Vision-Language-Action Reasoning" />
            </div>
          </div>
        </section>

        <section className={styles.agentsSection}>
          <div className="container">
            <Heading as="h2" className="text--center">Intelligent Agents</Heading>
            <div className={styles.agentsGrid}>
              <AgentCard name="Textbook Architect" role="Designing the knowledge structure" />
              <AgentCard name="Physical AI Professor" role="Guiding students through complex concepts" />
              <AgentCard name="Futuristic UI Designer" role="Crafting immersive user experiences" />
            </div>
          </div>
        </section>
      </main>
      <FloatingChatbotButton />
    </Layout>
  );
}



