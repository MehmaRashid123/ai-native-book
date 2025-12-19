import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface Message {
  text: string;
  sender: 'user' | 'bot';
  sources?: string[];
}

export default function ChatbotUI({ isOpen, onClose }: { isOpen: boolean; onClose: () => void }) {
  const [input, setInput] = useState('');
  const [messages, setMessage] = useState<Message[]>([
    { text: "Hello! I am your Physical AI assistant. How can I help you with the book today?", sender: 'bot' }
  ]);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');
    setMessage(prev => [...prev, { text: userMessage, sender: 'user' }]);
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: userMessage }),
      });

      if (!response.ok) throw new Error('Failed to connect to chatbot server');

      const data = await response.json();
      setMessage(prev => [...prev, { 
        text: data.answer, 
        sender: 'bot', 
        sources: data.sources 
      }]);
    } catch (error) {
      setMessage(prev => [...prev, { 
        text: "I'm having trouble connecting to my brain right now. Please make sure the backend server is running.", 
        sender: 'bot' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className={clsx(styles.chatbotWindow, 'glassmorphism-panel')}>
      <div className={styles.header}>
        <div className={styles.title}>AI Book Assistant</div>
        <button className={styles.closeButton} onClick={onClose}>&times;</button>
      </div>
      
      <div className={styles.messages}>
        {messages.map((m, i) => (
          <div key={i} className={clsx(styles.message, styles[m.sender])}>
            <div className={styles.messageText}>{m.text}</div>
            {m.sources && m.sources.length > 0 && (
              <div className={styles.sources}>
                <div className={styles.sourceLabel}>Sources:</div>
                {m.sources.map((s, si) => (
                  <a key={si} href={s} target="_blank" rel="noopener noreferrer" className={styles.sourceLink}>
                    View Page
                  </a>
                ))}
              </div>
            )}
          </div>
        ))}
        {isLoading && <div className={clsx(styles.message, styles.bot)}>Thinking...</div>}
        <div ref={messagesEndRef} />
      </div>

      <div className={styles.inputArea}>
        <input
          type="text"
          placeholder="Ask a question..."
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && handleSend()}
        />
        <button onClick={handleSend} disabled={isLoading}>Send</button>
      </div>
    </div>
  );
}
