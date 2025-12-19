import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface ChatBotContextProps {
  topic: string;
}

const ChatBotContext: React.FC<ChatBotContextProps> = ({ topic }) => {
  return (
    <div className={clsx(styles.chatBotContext, 'card padding--md')}>
      <p>Chatbot Context for: <strong>{topic}</strong> (Placeholder)</p>
      {/* Here you would integrate your actual chatbot */}
    </div>
  );
};

export default ChatBotContext;
