import React, { useState, useRef, useEffect } from 'react';
import './FloatingChatbot.css';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { text: "Hello! I'm your Book AI Assistant. I can answer questions about the book content. What would you like to know?", isUser: false }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Close chat when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (chatContainerRef.current && !chatContainerRef.current.contains(event.target) && isOpen) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

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
      const response = await fetch('https://ayeshaexe-deploy-backend.hf.space/query', {
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
    <>
      {/* Floating Chat Icon */}
      <div className="floating-chat-icon" onClick={() => setIsOpen(!isOpen)}>
        <div className="chat-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H16.5L15 20.5L13.5 17H5C4.46957 17 3.96086 16.7893 3.58579 16.4142C3.21071 16.0391 3 15.5304 3 15V6C3 5.46957 3.21071 4.96086 3.58579 4.58579C3.96086 4.21071 4.46957 4 5 4H19C19.5304 4 20.0391 4.21071 20.4142 4.58579C20.7893 4.96086 21 5.46957 21 6V15Z" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M12 8H12.01" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M8 8H8.01" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M16 8H16.01" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </div>
        {isOpen && <span className="chat-unread-dot"></span>}
      </div>

      {/* Chatbot Popup */}
      {isOpen && (
        <div className="floating-chat-container" ref={chatContainerRef}>
          <div className="chat-header">
            <h3>📚 Book AI Assistant</h3>
            <button className="close-btn" onClick={() => setIsOpen(false)}>
              ×
            </button>
          </div>

          <div className="chat-messages">
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

          <div className="chat-input">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the book..."
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
            >
              {isLoading ? '...' : '→'}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;