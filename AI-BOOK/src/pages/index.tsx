import FloatingChatbotButton from '@site/src/components/FloatingChatbotButton'; // Import FloatingChatbotButton
import { Button, Group, Text, List, Card } from '@mantine/core'; // Import Mantine Components

import styles from './index.module.css';

function HeroSection() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className={styles.heroTitle}>
          PHYSICAL AI
        </Heading>
        <p className={styles.heroSubtitle}>Embodied Intelligence in the Physical World</p>
        <Button variant="filled" color="neonCyan" size="xl" onClick={() => (window.location.href = '/docs/intro')} className={styles.heroButton} mt="xl">
          Start Learning (Mantine)
        </Button>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  // const {siteConfig} = useDocusaurusContext(); // Temporarily removed for debugging
  const siteConfig = { title: "My Site", tagline: "My Tagline" }; // Hardcoded placeholder for now
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

        <section className={clsx('margin-top--lg', 'text--center', styles.startLearningSection)}>
          <div className="container">
            <Button variant="filled" color="neonCyan" size="xl" onClick={() => (window.location.href = '/docs/intro')}>
              Start Learning (Mantine)
            </Button>
          </div>
        </section>


      </main>
      <FloatingChatbotButton />
    </Layout>
  );
}