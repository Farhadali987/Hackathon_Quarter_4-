import React, { useState, ReactNode } from 'react';
import styles from './styles.module.css';
import { useUser } from '@site/src/context/UserContext';

export default function Signup(): ReactNode {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const { login } = useUser();

  const handleSignup = (e: React.FormEvent) => {
    e.preventDefault();
    // TODO: Implement actual signup logic with better-auth
    const userProfile = { email, softwareBackground, hardwareBackground };
    login(userProfile);
    console.log('Signing up and logging in with:', userProfile);
    alert('You have been signed up and logged in!');
  };

  return (
    <div className={styles.authContainer}>
      <h3>Sign Up</h3>
      <form onSubmit={handleSignup} className={styles.authForm}>
        <div className={styles.inputGroup}>
          <label htmlFor="signup-email">Email</label>
          <input
            type="email"
            id="signup-email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
        </div>
        <div className={styles.inputGroup}>
          <label htmlFor="signup-password">Password</label>
          <input
            type="password"
            id="signup-password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
        </div>
        <div className={styles.inputGroup}>
          <label htmlFor="software-background">Software Background</label>
          <textarea
            id="software-background"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
            placeholder="e.g., Python, C++, React, ROS"
            required
          />
        </div>
        <div className={styles.inputGroup}>
          <label htmlFor="hardware-background">Hardware Background</label>
          <textarea
            id="hardware-background"
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
            placeholder="e.g., Arduino, Raspberry Pi, NVIDIA Jetson, Robotics"
            required
          />
        </div>
        <button type="submit" className="button button--primary">Sign Up</button>
      </form>
    </div>
  );
}
