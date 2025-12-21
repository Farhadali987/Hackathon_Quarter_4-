import React, { useState, useEffect } from 'react';
import styles from './LabGuidance.module.css';

const LabGuidance = ({ labId }) => {
  const [lab, setLab] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (labId) {
      fetchLabDetails();
    } else {
      // If no specific lab ID provided, fetch a list of available labs
      fetchAvailableLabs();
    }
  }, [labId]);

  const fetchLabDetails = async () => {
    try {
      setLoading(true);
      setError(null);
      
      const response = await fetch(`/api/labs/${labId}`);
      
      if (!response.ok) {
        throw new Error(`Failed to fetch lab: ${response.status}`);
      }
      
      const data = await response.json();
      setLab(data);
    } catch (err) {
      setError(err.message);
      console.error('Error fetching lab details:', err);
    } finally {
      setLoading(false);
    }
  };

  const fetchAvailableLabs = async () => {
    try {
      setLoading(true);
      setError(null);
      
      const response = await fetch('/api/labs');
      
      if (!response.ok) {
        throw new Error(`Failed to fetch labs: ${response.status}`);
      }
      
      const data = await response.json();
      // For this example, we'll just show the first lab
      if (data.labs && data.labs.length > 0) {
        setLab(data.labs[0]);
      }
    } catch (err) {
      setError(err.message);
      console.error('Error fetching available labs:', err);
    } finally {
      setLoading(false);
    }
  };

  const startLab = () => {
    // In a real implementation, this would start the lab
    console.log(`Starting lab: ${lab.title}`);
    alert(`Starting lab: ${lab.title}. Follow the instructions in the console.`);
  };

  if (loading) {
    return (
      <div className={styles.labGuidanceContainer}>
        <div className={styles.loadingSpinner}>Loading lab guidance...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.labGuidanceContainer}>
        <div className={styles.errorContainer}>
          <h3>Error Loading Lab</h3>
          <p>{error}</p>
          <button onClick={labId ? fetchLabDetails : fetchAvailableLabs} className={styles.retryButton}>
            Retry
          </button>
        </div>
      </div>
    );
  }

  if (!lab) {
    return (
      <div className={styles.labGuidanceContainer}>
        <div className={styles.noLabContainer}>
          <h3>No Lab Available</h3>
          <p>There are no labs available at this time.</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.labGuidanceContainer}>
      <header className={styles.labHeader}>
        <h1 className={styles.labTitle}>{lab.title}</h1>
        <div className={styles.labMetadata}>
          <span className={`${styles.badge} ${styles.typeBadge}`}>{lab.type}</span>
          <span className={`${styles.badge} ${styles.difficultyBadge}`}>{lab.difficulty}</span>
          <span className={`${styles.badge} ${styles.durationBadge}`}>{lab.duration} mins</span>
        </div>
      </header>

      <section className={styles.labDescription}>
        <h2>Description</h2>
        <p>{lab.description}</p>
      </section>

      <section className={styles.labRequirements}>
        <h2>Requirements</h2>
        <ul>
          {lab.requirements && lab.requirements.length > 0 ? (
            lab.requirements.map((req, index) => (
              <li key={index}>{req}</li>
            ))
          ) : (
            <li>No specific requirements listed</li>
          )}
        </ul>
      </section>

      <section className={styles.labObjectives}>
        <h2>Learning Objectives</h2>
        <ul>
          {lab.learningObjectives && lab.learningObjectives.length > 0 ? (
            lab.learningObjectives.map((objective, index) => (
              <li key={index}>{objective}</li>
            ))
          ) : (
            <li>No specific learning objectives listed</li>
          )}
        </ul>
      </section>

      <section className={styles.labInstructions}>
        <h2>Instructions</h2>
        <div className={styles.instructionsContent}>
          {/* Render instructions as markdown or plain text */}
          {lab.instructions ? (
            <pre className={styles.instructionsText}>{lab.instructions}</pre>
          ) : (
            <p>No instructions available.</p>
          )}
        </div>
      </section>

      <footer className={styles.labActions}>
        <button onClick={startLab} className={styles.startLabButton}>
          Start Lab
        </button>
        <button className={styles.saveProgressButton}>
          Save Progress
        </button>
      </footer>
    </div>
  );
};

export default LabGuidance;