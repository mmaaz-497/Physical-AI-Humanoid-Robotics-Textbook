import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/introduction/what-is-physical-ai">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

const ModuleList = [
  {
    title: 'Introduction to Physical AI',
    description:
      'Understand the foundations of Physical AI, embodied intelligence, and how it differs from digital AI. Explore humanoid robotics systems and architecture.',
    link: '/docs/introduction/what-is-physical-ai',
  },
  {
    title: 'Setup Guides',
    description:
      'Get started with Digital Twin workstations, Physical AI edge kits (Jetson), and cloud-native development environments.',
    link: '/docs/setup-guides/digital-twin-workstation',
  },
  {
    title: 'ROS 2 Fundamentals',
    description:
      'Master Robot Operating System 2 (ROS 2) - nodes, topics, services, actions, URDF modeling, and package management.',
    link: '/docs/ros2/introduction-to-ros2',
  },
  {
    title: 'Digital Twin Simulations',
    description:
      'Build realistic robot simulations in Gazebo, create sensor models, design environments, and integrate Unity with ROS.',
    link: '/docs/digital-twin/digital-twin-systems',
  },
  {
    title: 'NVIDIA Isaac Platform',
    description:
      'Explore Isaac Sim for high-fidelity simulation, perception pipelines, VSLAM navigation, synthetic data generation, and sim-to-real deployment.',
    link: '/docs/isaac/isaac-platform-intro',
  },
  {
    title: 'VLA & Humanoid Control',
    description:
      'Integrate Vision-Language-Action systems with humanoid robots. Learn voice-to-action, LLM planning, vision pipelines, kinematics, and manipulation.',
    link: '/docs/vla-humanoids/vla-introduction',
  },
  {
    title: 'References & Resources',
    description:
      'Access robotics glossary, ROS 2 cheatsheets, Isaac Sim commands, and downloadable resources for hands-on learning.',
    link: '/docs/references/robotics-glossary',
  },
];

function ModuleCard({title, description, link}) {
  return (
    <div className={clsx('col col--4', styles.moduleCard)}>
      <div className="module-card">
        <h3>{title}</h3>
        <p>{description}</p>
        <Link to={link}>
          Explore →
        </Link>
      </div>
    </div>
  );
}

function HomepageModules() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <h2 className={styles.modulesTitle}>Learning Modules</h2>
        <div className="row module-cards">
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageModules />
      </main>
    </Layout>
  );
}
