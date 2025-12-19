import React, { useState } from 'react';
import clsx from 'clsx';
import ChatbotUI from '../ChatbotUI';
import styles from './styles.module.css';

export default function FloatingChatbotButton(): JSX.Element {
  const [isChatOpen, setIsChatOpen] = useState(false);

  return (
    <>
      <div 
        className={clsx(styles.floatingChatbotButton, 'glassmorphism-panel')}
        onClick={() => setIsChatOpen(!isChatOpen)}
      >
        <div className={styles.icon}>{isChatOpen ? '×' : '🤖'}</div>
      </div>
      <ChatbotUI isOpen={isChatOpen} onClose={() => setIsChatOpen(false)} />
    </>
  );
}
