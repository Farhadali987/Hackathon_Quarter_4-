import React, { useState, useEffect } from 'react';
import styles from './Textbook.module.css';

const Textbook = ({ contentId }) => {
  const [content, setContent] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (contentId) {
      fetchContent();
    }
  }, [contentId]);

  const fetchContent = async () => {
    try {
      setLoading(true);
      setError(null);
      
      const response = await fetch(`/api/textbook/content/${contentId}`);
      
      if (!response.ok) {
        throw new Error(`Failed to fetch content: ${response.status}`);
      }
      
      const data = await response.json();
      setContent(data);
    } catch (err) {
      setError(err.message);
      console.error('Error fetching textbook content:', err);
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <div className={styles.textbookContainer}>
        <div className={styles.loadingSpinner}>Loading textbook content...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.textbookContainer}>
        <div className={styles.errorContainer}>
          <h3>Error Loading Content</h3>
          <p>{error}</p>
          <button onClick={fetchContent} className={styles.retryButton}>
            Retry
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.textbookContainer}>
      {content ? (
        <div className={styles.contentWrapper}>
          <header className={styles.contentHeader}>
            <h1 className={styles.title}>{content.title}</h1>
            <div className={styles.metadata}>
              <span className={styles.topic}>Topic: {content.topic}</span>
              <span className={styles.level}>Level: {content.level}</span>
              <span className={styles.language}>Language: {content.language}</span>
            </div>
          </header>
          
          <main className={styles.contentBody}>
            <div 
              className={styles.textContent}
              dangerouslySetInnerHTML={{ __html: content.content }}
            />
          </main>
          
          <footer className={styles.contentFooter}>
            <div className={styles.contentInfo}>
              <p>Created: {new Date(content.createdAt).toLocaleDateString()}</p>
              <p>Last Updated: {new Date(content.updatedAt).toLocaleDateString()}</p>
            </div>
          </footer>
        </div>
      ) : (
        <div className={styles.placeholder}>
          <h3>Select a textbook chapter to view</h3>
          <p>Choose from the textbook navigation to see content here.</p>
        </div>
      )}
    </div>
  );
};

// Component for displaying a list of textbook contents
export const TextbookList = ({ 
  topic, 
  level, 
  language, 
  contentType, 
  parentId,
  onItemClick 
}) => {
  const [contents, setContents] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    fetchContents();
  }, [topic, level, language, contentType, parentId]);

  const fetchContents = async () => {
    try {
      setLoading(true);
      setError(null);
      
      // Build query parameters
      const params = new URLSearchParams();
      if (topic) params.append('topic', topic);
      if (level) params.append('level', level);
      if (language) params.append('language', language);
      if (contentType) params.append('contentType', contentType);
      if (parentId) params.append('parentId', parentId);
      
      const queryString = params.toString();
      const url = `/api/textbook/content${queryString ? `?${queryString}` : ''}`;
      
      const response = await fetch(url);
      
      if (!response.ok) {
        throw new Error(`Failed to fetch contents: ${response.status}`);
      }
      
      const data = await response.json();
      setContents(data.content);
    } catch (err) {
      setError(err.message);
      console.error('Error fetching textbook contents:', err);
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return <div className={styles.loadingSpinner}>Loading textbook contents...</div>;
  }

  if (error) {
    return (
      <div className={styles.errorContainer}>
        <h3>Error Loading Contents</h3>
        <p>{error}</p>
        <button onClick={fetchContents} className={styles.retryButton}>
          Retry
        </button>
      </div>
    );
  }

  return (
    <div className={styles.textbookListContainer}>
      <h2 className={styles.listTitle}>Textbook Contents</h2>
      
      {contents.length === 0 ? (
        <div className={styles.noContents}>
          <p>No textbook contents found matching the criteria.</p>
        </div>
      ) : (
        <ul className={styles.contentsList}>
          {contents.map((item) => (
            <li 
              key={item.id} 
              className={styles.contentItem}
              onClick={() => onItemClick && onItemClick(item.id)}
            >
              <h3 className={styles.contentTitle}>{item.title}</h3>
              <div className={styles.contentMeta}>
                <span className={styles.topic}>{item.topic}</span>
                <span className={styles.level}>{item.level}</span>
              </div>
              <p className={styles.contentPreview}>
                {item.content.substring(0, 150)}...
              </p>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
};

export default Textbook;