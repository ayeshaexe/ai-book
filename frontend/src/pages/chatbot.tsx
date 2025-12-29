import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './chatbot.css';

export default function ChatbotPage() {
  const { siteConfig } = useDocusaurusContext();
  const [messages, setMessages] = useState([
    { text: "Hello! I'm your Book AI Assistant. I can answer questions about the book content. What would you like to know?", isUser: false }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    const message = inputValue.trim();
    if (!message) return;

    // Add user message
    const newMessage = { text: message, isUser: true };
    setMessages(prev => [...prev, newMessage]);
    setInputValue('');
    setIsLoading(true);
    setError('');

    try {
      const response = await fetch('http://127.0.0.1:8000/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: message,
          context: "User question about book content",
          user_id: "web_user_" + Date.now()
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Add bot response
      setMessages(prev => [...prev, { text: data.answer, isUser: false }]);
    } catch (error) {
      console.error('Error:', error);
      setError('Error: ' + error.message);
      setMessages(prev => [...prev, {
        text: "Sorry, I encountered an issue processing your request. Please try again.",
        isUser: false
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <Layout
      title={`AI Assistant - ${siteConfig.title}`}
      description="AI Chatbot to ask questions about the book content">
      <div className="chatbot-container">
        <div className="chatbot-header">
          <h1>📚 {siteConfig.title} AI Assistant</h1>
          <p>Ask me anything about the book content</p>
        </div>

        <div className="chatbot-messages">
          {messages.map((msg, index) => (
            <div key={index} className={`message ${msg.isUser ? 'user-message' : 'bot-message'}`}>
              {msg.text}
            </div>
          ))}
          {isLoading && (
            <div className="message bot-message typing-indicator">
              AI is thinking...
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        {error && (
          <div className="error-message">
            {error}
          </div>
        )}

        <div className="chatbot-input">
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your question about the book..."
            disabled={isLoading}
          />
          <button
            onClick={sendMessage}
            disabled={isLoading || !inputValue.trim()}
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </div>
    </Layout>
  );
}