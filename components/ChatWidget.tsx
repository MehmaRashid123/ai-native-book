import React, { useState, useEffect, useRef } from 'react';

interface Message {
  text: string;
  sender: 'user' | 'bot';
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const API_BASE_URL = 'http://localhost:8000';

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  const getSelectedText = () => {
    if (window.getSelection) {
      return window.getSelection()?.toString();
    }
    return '';
  };

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = input;
    const selection = getSelectedText();
    const context = selection ? selection : undefined;

    const newUserMessage: Message = { text: userMessage, sender: 'user' };
    setMessages((prev) => [...prev, newUserMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMessage, context: context }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage: Message = { text: data.response, sender: 'bot' };
      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prev) => [...prev, { text: 'Sorry, I am unable to respond right now.', sender: 'bot' }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      sendMessage();
    }
  };

  // Styles
  const fabStyle: React.CSSProperties = {
    position: 'fixed',
    bottom: '24px',
    right: '24px',
    zIndex: 9999,
    width: '56px',
    height: '56px',
    borderRadius: '50%',
    backgroundColor: '#2563eb',
    color: 'white',
    border: 'none',
    boxShadow: '0 4px 6px rgba(0,0,0,0.3)',
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    transition: 'transform 0.2s',
  };

  const chatWindowStyle: React.CSSProperties = {
    position: 'fixed',
    bottom: '90px',
    right: '24px',
    zIndex: 9999,
    width: '350px',
    height: '500px',
    backgroundColor: 'white',
    borderRadius: '12px',
    boxShadow: '0 10px 25px rgba(0,0,0,0.2)',
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
    border: '1px solid #e5e7eb',
  };

  const headerStyle: React.CSSProperties = {
    backgroundColor: '#2563eb',
    color: 'white',
    padding: '16px',
    fontWeight: 'bold',
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
  };

  const messagesStyle: React.CSSProperties = {
    flex: 1,
    overflowY: 'auto',
    padding: '16px',
    backgroundColor: '#f3f4f6',
    display: 'flex',
    flexDirection: 'column',
    gap: '10px',
  };

  const inputAreaStyle: React.CSSProperties = {
    padding: '16px',
    borderTop: '1px solid #e5e7eb',
    backgroundColor: 'white',
    display: 'flex',
    gap: '10px',
  };

  const inputStyle: React.CSSProperties = {
    flex: 1,
    padding: '8px 12px',
    borderRadius: '20px',
    border: '1px solid #d1d5db',
    outline: 'none',
  };

  if (!isOpen) {
    return (
      <button style={fabStyle} onClick={() => setIsOpen(true)}>
        <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
      </button>
    );
  }

  return (
    <>
      <button style={fabStyle} onClick={() => setIsOpen(false)}>
        <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <line x1="18" y1="6" x2="6" y2="18"></line>
          <line x1="6" y1="6" x2="18" y2="18"></line>
        </svg>
      </button>

      <div style={chatWindowStyle}>
        <div style={headerStyle}>
          <span>Physical AI Tutor</span>
        </div>

        <div style={messagesStyle}>
          {messages.map((msg, index) => (
            <div
              key={index}
              style={{
                alignSelf: msg.sender === 'user' ? 'flex-end' : 'flex-start',
                backgroundColor: msg.sender === 'user' ? '#2563eb' : 'white',
                color: msg.sender === 'user' ? 'white' : '#1f2937',
                padding: '8px 12px',
                borderRadius: '12px',
                maxWidth: '80%',
                boxShadow: '0 1px 2px rgba(0,0,0,0.1)',
                borderBottomRightRadius: msg.sender === 'user' ? '0' : '12px',
                borderBottomLeftRadius: msg.sender === 'bot' ? '0' : '12px',
              }}
            >
              {msg.text}
            </div>
          ))}
          {isLoading && (
            <div style={{ alignSelf: 'flex-start', color: '#6b7280', fontSize: '12px', marginLeft: '8px' }}>
              Thinking...
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div style={inputAreaStyle}>
          <input
            type="text"
            placeholder="Ask a question..."
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyPress}
            style={inputStyle}
            disabled={isLoading}
          />
          <button 
            onClick={() => sendMessage()} 
            disabled={isLoading}
            style={{
              backgroundColor: '#2563eb',
              color: 'white',
              border: 'none',
              borderRadius: '50%',
              width: '36px',
              height: '36px',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center'
            }}
          >
            <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
            </svg>
          </button>
        </div>
      </div>
    </>
  );
};

export default ChatWidget;