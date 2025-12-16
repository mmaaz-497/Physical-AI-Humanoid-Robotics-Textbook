/**
 * Sign In Page for Physical AI Humanoid Robotics Book
 * Allows existing users to authenticate and access their personalized content
 */

import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { authClient } from '../../lib/auth-client';
import styles from './auth.module.css';

export default function SignInPage() {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (!formData.email || !formData.password) {
      setError('Please enter both email and password');
      return;
    }

    setLoading(true);

    try {
      // Sign in using Better Auth
      await authClient.signIn.email({
        email: formData.email,
        password: formData.password,
        callbackURL: '/docs/introduction/what-is-physical-ai',
      });

      // Redirect to docs after successful sign in
      window.location.href = '/docs/introduction/what-is-physical-ai';
    } catch (err: any) {
      console.error('Sign in error:', err);
      setError(err.message || 'Invalid email or password. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign In">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Welcome Back!</h1>
          <p className={styles.authSubtitle}>
            Sign in to access your personalized learning experience
          </p>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            <div className={styles.formGroup}>
              <label htmlFor="email" className={styles.label}>
                Email
              </label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleInputChange}
                className={styles.input}
                placeholder="your.email@example.com"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password" className={styles.label}>
                Password
              </label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleInputChange}
                className={styles.input}
                placeholder="Enter your password"
                required
              />
            </div>

            {error && (
              <div className={styles.errorMessage}>
                {error}
              </div>
            )}

            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? 'Signing In...' : 'Sign In'}
            </button>

            <p className={styles.authFooter}>
              Don't have an account?{' '}
              <Link to="/auth/signup" className={styles.authLink}>
                Sign Up
              </Link>
            </p>

            <p className={styles.authFooter}>
              <Link to="/auth/forgot-password" className={styles.authLink}>
                Forgot your password?
              </Link>
            </p>
          </form>
        </div>
      </div>
    </Layout>
  );
}
