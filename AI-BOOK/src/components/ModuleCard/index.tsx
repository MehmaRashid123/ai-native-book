import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface ModuleCardProps {
  title: string;
  description: string;
}

export default function ModuleCard({ title, description }: ModuleCardProps): JSX.Element {
  return (
    <div className={clsx(styles.moduleCard, 'glassmorphism-panel')}>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}
