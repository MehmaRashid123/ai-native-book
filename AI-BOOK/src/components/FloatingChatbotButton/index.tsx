import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export default function FloatingChatbotButton(): JSX.Element {
  return (
    <div className={clsx(styles.floatingChatbotButton, 'glassmorphism-panel')}>
      <span className={styles.label}>AI Assistant</span>
    </div>
  );
}
