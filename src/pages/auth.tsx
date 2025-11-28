import React, { useState, ReactNode } from 'react';
import Layout from '@theme/Layout';
import Signin from '@site/src/components/Auth/Signin';
import Signup from '@site/src/components/Auth/Signup';
import styles from './auth.module.css';
import clsx from 'clsx';

export default function AuthPage(): ReactNode {
  const [activeTab, setActiveTab] = useState<'signin' | 'signup'>('signin');

  return (
    <Layout title="Authentication" description="Sign in or sign up for an account.">
      <main className="container margin-vert--lg">
        <div className={styles.authPageContainer}>
          <div className={styles.tabContainer}>
            <button
              className={clsx(styles.tabButton, { [styles.activeTab]: activeTab === 'signin' })}
              onClick={() => setActiveTab('signin')}
            >
              Sign In
            </button>
            <button
              className={clsx(styles.tabButton, { [styles.activeTab]: activeTab === 'signup' })}
              onClick={() => setActiveTab('signup')}
            >
              Sign Up
            </button>
          </div>
          <div className={styles.tabContent}>
            {activeTab === 'signin' ? <Signin /> : <Signup />}
          </div>
        </div>
      </main>
    </Layout>
  );
}
