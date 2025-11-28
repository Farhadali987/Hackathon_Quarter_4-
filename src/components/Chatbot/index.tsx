import React, { useState, useRef, useEffect, ReactNode } from 'react';
import styles from './styles.module.css';
import clsx from 'clsx';

type Message = {
  text: string;
  sender: 'user' | 'bot';
};

export default function Chatbot(): ReactNode {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const chatHistoryRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (chatHistoryRef.current) {
      chatHistoryRef.current.scrollTop = chatHistoryRef.current.scrollHeight;
    }
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && messages.length === 0) {
      setMessages([{ text: "Hello! I'm the textbook assistant. How can I help you?", sender: 'bot' }]);
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(e.target.value);
  };

  const handleSendMessage = async () => {
    if (inputValue.trim() === '') return;

    const userMessage: Message = { text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    // Mock bot response
    setTimeout(() => {
      const botMessage: Message = { text: `You said: "${inputValue}". The real bot is not connected yet.`, sender: 'bot' };
      setMessages(prev => [...prev, botMessage]);
    }, 500);

    // TODO: Replace mock response with actual API call to the backend
    /*
    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: inputValue }),
      });
      const data = await response.json();
      const botMessage: Message = { text: data.response, sender: 'bot' };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error("Error fetching chat response:", error);
      const errorMessage: Message = { text: "Sorry, I'm having trouble connecting to the server.", sender: 'bot' };
      setMessages(prev => [...prev, errorMessage]);
    }
    */
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleSendMessage();
    }
  };

  return (
    <>
      <button className={styles.chatToggleButton} onClick={toggleChat}>
        {isOpen ? '✖' : '🤖'}
      </button>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Robotics Assistant</h3>
          </div>
          <div className={styles.chatHistory} ref={chatHistoryRef}>
            {messages.map((msg, index) => (
              <div key={index} className={clsx(styles.chatMessage, styles[msg.sender])}>
                <p>{msg.text}</p>
              </div>
            ))}
          </div>
          <div className={styles.chatInputContainer}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              className={styles.chatInput}
            />
            <button onClick={handleSendMessage} className={styles.sendButton}>➤</button>
          </div>
        </div>
      )}
    </>
  );
}
