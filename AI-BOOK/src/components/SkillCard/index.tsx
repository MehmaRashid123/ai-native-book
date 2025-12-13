import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface SkillCardProps {
  title: string;
}

export default function SkillCard({ title }: SkillCardProps): JSX.Element {
  return (
    <div className={clsx(styles.skillCard, 'glassmorphism-panel')}>
      <h3>{title}</h3>
    </div>
  );
}
