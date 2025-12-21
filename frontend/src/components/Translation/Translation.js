import React, { useState, useEffect } from 'react';
import styles from './Translation.module.css';

const Translation = ({ onLanguageChange }) => {
  const [availableLanguages, setAvailableLanguages] = useState([]);
  const [currentLanguage, setCurrentLanguage] = useState('en');
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    fetchLanguages();
  }, []);

  const fetchLanguages = async () => {
    try {
      setLoading(true);
      setError(null);
      
      const response = await fetch('/api/translation/available');
      
      if (!response.ok) {
        throw new Error(`Failed to fetch languages: ${response.status}`);
      }
      
      const data = await response.json();
      setAvailableLanguages(data.languages);
      
      // Set the current language based on user preference or browser default
      const userLang = localStorage.getItem('preferredLanguage') || 
                      navigator.language.split('-')[0] || 'en';
      setCurrentLanguage(userLang);
      
      // Notify parent component of initial language
      if (onLanguageChange) {
        onLanguageChange(userLang);
      }
    } catch (err) {
      setError(err.message);
      console.error('Error fetching available languages:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleLanguageChange = async (newLanguage) => {
    try {
      // In a real implementation, we would update user preferences on the backend
      // and potentially reload content in the new language
      
      // Update the current language
      setCurrentLanguage(newLanguage);
      
      // Save user preference
      localStorage.setItem('preferredLanguage', newLanguage);
      
      // Notify parent component of language change
      if (onLanguageChange) {
        onLanguageChange(newLanguage);
      }
      
      // In a real app, we would trigger content re-rendering here
      console.log(`Language changed to: ${newLanguage}`);
    } catch (err) {
      setError(err.message);
      console.error('Error changing language:', err);
    }
  };

  if (loading) {
    return (
      <div className={styles.translationContainer}>
        <div className={styles.loadingSpinner}>Loading language options...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.translationContainer}>
        <div className={styles.errorContainer}>
          <h3>Error Loading Languages</h3>
          <p>{error}</p>
          <button onClick={fetchLanguages} className={styles.retryButton}>
            Retry
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.translationContainer}>
      <label htmlFor="languageSelector" className={styles.label}>
        Select Language:
      </label>
      
      <select
        id="languageSelector"
        value={currentLanguage}
        onChange={(e) => handleLanguageChange(e.target.value)}
        className={styles.languageSelector}
      >
        {availableLanguages
          .filter(lang => lang.isSupported)  // Only show supported languages
          .map((lang) => (
            <option key={lang.code} value={lang.code}>
              {lang.name} ({lang.code})
            </option>
          ))}
      </select>
      
      <div className={styles.languageInfo}>
        Current language: <strong>{currentLanguage}</strong>
      </div>
    </div>
  );
};

export default Translation;