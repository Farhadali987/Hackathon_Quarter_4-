import React, { useState, useEffect } from 'react';
import styles from './Personalization.module.css';

const Personalization = ({ userId }) => {
  const [userPreferences, setUserPreferences] = useState({
    preferredLanguage: 'en',
    learningLevel: 'beginner',
    preferredTopics: [],
    accessibilityOptions: {
      fontSize: 'medium',
      contrast: 'normal',
      dyslexiaFriendly: false
    },
    notificationPreferences: {
      email: true,
      push: false,
      progressUpdates: true
    }
  });

  const [loading, setLoading] = useState(true);
  const [saved, setSaved] = useState(false);

  useEffect(() => {
    loadUserPreferences();
  }, [userId]);

  const loadUserPreferences = async () => {
    try {
      setLoading(true);
      // In a real implementation, this would fetch user preferences from the backend
      // For this example, we'll use the default preferences
    } catch (error) {
      console.error('Error loading user preferences:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleChange = (section, key, value) => {
    setUserPreferences(prev => ({
      ...prev,
      [section]: {
        ...prev[section],
        [key]: value
      }
    }));
  };

  const handleTopicToggle = (topic) => {
    setUserPreferences(prev => {
      const topics = prev.preferredTopics.includes(topic)
        ? prev.preferredTopics.filter(t => t !== topic)
        : [...prev.preferredTopics, topic];
      
      return {
        ...prev,
        preferredTopics: topics
      };
    });
  };

  const handleSave = async () => {
    try {
      // In a real implementation, this would save preferences to the backend
      console.log('Saving user preferences:', userPreferences);
      
      // Show saved confirmation
      setSaved(true);
      setTimeout(() => setSaved(false), 3000);
      
      // In a real app, we would make an API call:
      /*
      const response = await fetch(`/api/user/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          personalizationSettings: userPreferences
        })
      });
      
      if (!response.ok) {
        throw new Error('Failed to save preferences');
      }
      */
    } catch (error) {
      console.error('Error saving preferences:', error);
      alert('Error saving preferences. Please try again.');
    }
  };

  if (loading) {
    return (
      <div className={styles.personalizationContainer}>
        <div className={styles.loadingSpinner}>Loading personalization settings...</div>
      </div>
    );
  }

  return (
    <div className={styles.personalizationContainer}>
      <header className={styles.personalizationHeader}>
        <h1>Personalization Settings</h1>
        <p>Customize your learning experience</p>
      </header>

      <div className={styles.settingsForm}>
        {saved && (
          <div className={styles.successMessage}>
            Preferences saved successfully!
          </div>
        )}

        {/* Language Preferences */}
        <section className={styles.settingSection}>
          <h2>Language Preferences</h2>
          <div className={styles.formGroup}>
            <label htmlFor="preferredLanguage">Preferred Language</label>
            <select
              id="preferredLanguage"
              value={userPreferences.preferredLanguage}
              onChange={(e) => handleChange('preferredLanguage', 'preferredLanguage', e.target.value)}
              className={styles.selectField}
            >
              <option value="en">English</option>
              <option value="ur">Urdu</option>
              <option value="es">Spanish</option>
              <option value="fr">French</option>
              <option value="de">German</option>
            </select>
            <p className={styles.helpText}>Select your preferred language for the interface and content</p>
          </div>
        </section>

        {/* Learning Level */}
        <section className={styles.settingSection}>
          <h2>Learning Level</h2>
          <div className={styles.formGroup}>
            <label htmlFor="learningLevel">Current Level</label>
            <select
              id="learningLevel"
              value={userPreferences.learningLevel}
              onChange={(e) => handleChange('learningLevel', 'learningLevel', e.target.value)}
              className={styles.selectField}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
            <p className={styles.helpText}>Adjusts the content difficulty based on your experience</p>
          </div>
        </section>

        {/* Preferred Topics */}
        <section className={styles.settingSection}>
          <h2>Preferred Topics</h2>
          <p className={styles.sectionDescription}>Select topics you're most interested in to customize your learning path</p>
          
          <div className={styles.topicsGrid}>
            {[
              'ROS 2', 'Gazebo', 'NVIDIA Isaac', 'VLA Models', 
              'Humanoid Robotics', 'Digital Twin', 'Conversational AI', 
              'Machine Learning', 'Computer Vision', 'Control Systems'
            ].map(topic => (
              <button
                key={topic}
                type="button"
                className={`${styles.topicButton} ${
                  userPreferences.preferredTopics.includes(topic) ? styles.selected : ''
                }`}
                onClick={() => handleTopicToggle(topic)}
              >
                {topic}
              </button>
            ))}
          </div>
        </section>

        {/* Accessibility Options */}
        <section className={styles.settingSection}>
          <h2>Accessibility Options</h2>
          
          <div className={styles.accessibilityOptions}>
            <div className={styles.formGroup}>
              <label htmlFor="fontSize">Font Size</label>
              <select
                id="fontSize"
                value={userPreferences.accessibilityOptions.fontSize}
                onChange={(e) => handleChange('accessibilityOptions', 'fontSize', e.target.value)}
                className={styles.selectField}
              >
                <option value="small">Small</option>
                <option value="medium">Medium</option>
                <option value="large">Large</option>
                <option value="xlarge">Extra Large</option>
              </select>
            </div>
            
            <div className={styles.formGroup}>
              <label htmlFor="contrast">Contrast Level</label>
              <select
                id="contrast"
                value={userPreferences.accessibilityOptions.contrast}
                onChange={(e) => handleChange('accessibilityOptions', 'contrast', e.target.value)}
                className={styles.selectField}
              >
                <option value="normal">Normal</option>
                <option value="high">High Contrast</option>
              </select>
            </div>
            
            <div className={styles.checkboxGroup}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={userPreferences.accessibilityOptions.dyslexiaFriendly}
                  onChange={(e) => handleChange('accessibilityOptions', 'dyslexiaFriendly', e.target.checked)}
                />
                <span>Dyslexia-friendly font</span>
              </label>
            </div>
          </div>
        </section>

        {/* Notification Preferences */}
        <section className={styles.settingSection}>
          <h2>Notification Preferences</h2>
          
          <div className={styles.notificationOptions}>
            <div className={styles.checkboxGroup}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={userPreferences.notificationPreferences.email}
                  onChange={(e) => handleChange('notificationPreferences', 'email', e.target.checked)}
                />
                <span>Email notifications</span>
              </label>
            </div>
            
            <div className={styles.checkboxGroup}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={userPreferences.notificationPreferences.push}
                  onChange={(e) => handleChange('notificationPreferences', 'push', e.target.checked)}
                />
                <span>Push notifications</span>
              </label>
            </div>
            
            <div className={styles.checkboxGroup}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={userPreferences.notificationPreferences.progressUpdates}
                  onChange={(e) => handleChange('notificationPreferences', 'progressUpdates', e.target.checked)}
                />
                <span>Progress updates</span>
              </label>
            </div>
          </div>
        </section>

        {/* Save Button */}
        <div className={styles.formActions}>
          <button 
            type="button" 
            className={styles.saveButton}
            onClick={handleSave}
          >
            Save Preferences
          </button>
        </div>
      </div>
    </div>
  );
};

export default Personalization;