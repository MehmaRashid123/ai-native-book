import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface AgentCardProps {
  name: string;
  role: string;
}

export default function AgentCard({ name, role }: AgentCardProps): JSX.Element {
  return (
    <div className={clsx(styles.agentCard, 'glassmorphism-panel')}>
      <h3>{name}</h3>
      <p>{role}</p>
    </div>
  );
}
