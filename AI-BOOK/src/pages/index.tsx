import FloatingChatbotButton from '@site/src/components/FloatingChatbotButton'; // Import FloatingChatbotButton
import { Button, Group, Text, Grid, List } from '@mantine/core'; // Import Mantine Components

import styles from './index.module.css';

function HeroSection() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Grid align="center" justify="space-between" gutter="xl">
          <Grid.Col span={{ base: 12, md: 6 }}>
            <Heading as="h1" className={styles.heroTitle}>
              PHYSICAL AI
            </Heading>
            <p className={styles.heroSubtitle}>Embodied Intelligence in the Physical World</p>
            <Button variant="filled" color="neonCyan" size="xl" onClick={() => (window.location.href = '/docs/intro')} className={styles.heroButton}>
              Start Learning (Mantine)
            </Button>
          </Grid.Col>
          <Grid.Col span={{ base: 12, md: 6 }} className={styles.heroImageColumn}>
            {/* Image background is applied to heroBanner, this column acts as a placeholder or can have overlay */}
          </Grid.Col>
        </Grid>
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
            <List
              spacing="md"
              size="lg"
              center
              withPadding
              className={styles.modulesList}
            >
              <List.Item>
                <Text color="neonCyan" fw={700}>ROS 2 Nervous System:</Text>
                <Text c="dimmed">Develop robust robot control architectures.</Text>
              </List.Item>
              <List.Item>
                <Text color="neonCyan" fw={700}>Digital Twin Simulation:</Text>
                <Text c="dimmed">Create and interact with virtual robot environments.</Text>
              </List.Item>
              <List.Item>
                <Text color="neonCyan" fw={700}>NVIDIA Isaac AI:</Text>
                <Text c="dimmed">Leverage advanced AI for robotic perception and control.</Text>
              </List.Item>
              <List.Item>
                <Text color="neonCyan" fw={700}>Vision-Language-Action:</Text>
                <Text c="dimmed">Integrate multimodal AI for intelligent robot behavior.</Text>
              </List.Item>
              <List.Item>
                <Text color="neonCyan" fw={700}>Capstone Project:</Text>
                <Text c="dimmed">Apply all learned skills to a real-world robotics challenge.</Text>
              </List.Item>
            </List>
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



