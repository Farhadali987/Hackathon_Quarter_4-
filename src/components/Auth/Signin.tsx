import React, { useState, ReactNode } from 'react';
import styles from './styles.module.css';

export default function Signin(): ReactNode {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleSignin = (e: React.FormEvent) => {
    e.preventDefault();
    // TODO: Implement actual signin logic with better-auth
    console.log('Signing in with:', { email, password });
    alert('Signin functionality not yet implemented.');
  };

  return (
    <div className={styles.authContainer}>
      <h3>Sign In</h3>
      <form onSubmit={handleSignin} className={styles.authForm}>
        <div className={styles.inputGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
        </div>
        <div className={styles.inputGroup}>
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
        </div>
        <button type="submit" className="button button--primary">Sign In</button>
      </form>
    </div>
  );
}
