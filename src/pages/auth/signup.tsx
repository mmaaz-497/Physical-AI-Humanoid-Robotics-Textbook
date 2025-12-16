/**
 * Signup Page for Physical AI Humanoid Robotics Book
 * Collects user account info and background information for personalization
 */

import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { authClient } from '../../lib/auth-client';
import styles from './auth.module.css';

export default function SignupPage() {
  // Form state
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
    // Software background
    experienceLevel: 'beginner' as 'beginner' | 'intermediate' | 'advanced',
    programmingLanguages: [] as string[],
    interestAreas: [] as string[],
    // Hardware background
    hardwareSkillLevel: 'beginner' as 'beginner' | 'intermediate' | 'advanced',
    hasHardwareExperience: false,
    hardwareDescription: '',
    // Professional info
    professionalRole: 'student' as 'student' | 'researcher' | 'engineer' | 'hobbyist' | 'other',
    roleOther: '',
    organization: '',
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);

  // Programming languages options
  const programmingLanguagesOptions = [
    'Python',
    'JavaScript/TypeScript',
    'C++',
    'C',
    'Java',
    'Rust',
    'Go',
    'MATLAB',
    'Other',
  ];

  // Interest areas options
  const interestAreasOptions = [
    'Web Development',
    'Artificial Intelligence',
    'Machine Learning',
    'Robotics',
    'IoT',
    'Computer Vision',
    'Backend Development',
    'Mobile Development',
    'Blockchain',
    'Embedded Systems',
  ];

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value, type } = e.target;

    if (type === 'checkbox') {
      const checked = (e.target as HTMLInputElement).checked;
      setFormData(prev => ({ ...prev, [name]: checked }));
    } else {
      setFormData(prev => ({ ...prev, [name]: value }));
    }
  };

  const handleMultiSelect = (category: 'programmingLanguages' | 'interestAreas', value: string) => {
    setFormData(prev => ({
      ...prev,
      [category]: prev[category].includes(value)
        ? prev[category].filter((item) => item !== value)
        : [...prev[category], value],
    }));
  };

  const validateForm = (): string | null => {
    if (!formData.email || !formData.email.includes('@')) {
      return 'Please enter a valid email address';
    }

    if (!formData.password || formData.password.length < 8) {
      return 'Password must be at least 8 characters long';
    }

    if (formData.password !== formData.confirmPassword) {
      return 'Passwords do not match';
    }

    if (!formData.name.trim()) {
      return 'Please enter your name';
    }

    if (formData.programmingLanguages.length === 0) {
      return 'Please select at least one programming language';
    }

    if (formData.interestAreas.length === 0) {
      return 'Please select at least one area of interest';
    }

    if (formData.professionalRole === 'other' && !formData.roleOther.trim()) {
      return 'Please specify your professional role';
    }

    return null;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    // Validate form
    const validationError = validateForm();
    if (validationError) {
      setError(validationError);
      return;
    }

    setLoading(true);

    try {
      // Sign up using Better Auth
      await authClient.signUp.email({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        // Better Auth additionalFields (from auth.ts config)
        experienceLevel: formData.experienceLevel,
        professionalRole: formData.professionalRole,
        roleOther: formData.professionalRole === 'other' ? formData.roleOther : undefined,
        organization: formData.organization || undefined,
        // Store additional background data as JSON in a custom field
        callbackURL: '/docs/introduction/what-is-physical-ai',
      });

      setSuccess(true);

      // Redirect after 2 seconds
      setTimeout(() => {
        window.location.href = '/docs/introduction/what-is-physical-ai';
      }, 2000);
    } catch (err: any) {
      console.error('Signup error:', err);
      setError(err.message || 'Failed to create account. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  if (success) {
    return (
      <Layout title="Sign Up Success">
        <div className={styles.authContainer}>
          <div className={styles.authCard}>
            <div className={styles.successMessage}>
              <h1>🎉 Welcome to Physical AI!</h1>
              <p>Your account has been created successfully.</p>
              <p>Redirecting to your personalized learning journey...</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign Up">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Create Your Account</h1>
          <p className={styles.authSubtitle}>
            Join the Physical AI learning community and get personalized content
          </p>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {/* Account Information */}
            <section className={styles.formSection}>
              <h2 className={styles.sectionTitle}>Account Information</h2>

              <div className={styles.formGroup}>
                <label htmlFor="name" className={styles.label}>Full Name *</label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  className={styles.input}
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="email" className={styles.label}>Email *</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  className={styles.input}
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password" className={styles.label}>Password *</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  className={styles.input}
                  minLength={8}
                  required
                />
                <small className={styles.helpText}>Minimum 8 characters</small>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="confirmPassword" className={styles.label}>Confirm Password *</label>
                <input
                  type="password"
                  id="confirmPassword"
                  name="confirmPassword"
                  value={formData.confirmPassword}
                  onChange={handleInputChange}
                  className={styles.input}
                  required
                />
              </div>
            </section>

            {/* Software Background */}
            <section className={styles.formSection}>
              <h2 className={styles.sectionTitle}>Software Background</h2>

              <div className={styles.formGroup}>
                <label htmlFor="experienceLevel" className={styles.label}>Experience Level *</label>
                <select
                  id="experienceLevel"
                  name="experienceLevel"
                  value={formData.experienceLevel}
                  onChange={handleInputChange}
                  className={styles.select}
                  required
                >
                  <option value="beginner">Beginner - New to programming</option>
                  <option value="intermediate">Intermediate - Some hands-on experience</option>
                  <option value="advanced">Advanced - Professional or research level</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.label}>Programming Languages * (Select all that apply)</label>
                <div className={styles.checkboxGrid}>
                  {programmingLanguagesOptions.map((lang) => (
                    <label key={lang} className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={formData.programmingLanguages.includes(lang)}
                        onChange={() => handleMultiSelect('programmingLanguages', lang)}
                        className={styles.checkbox}
                      />
                      <span>{lang}</span>
                    </label>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.label}>Areas of Interest * (Select all that apply)</label>
                <div className={styles.checkboxGrid}>
                  {interestAreasOptions.map((area) => (
                    <label key={area} className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={formData.interestAreas.includes(area)}
                        onChange={() => handleMultiSelect('interestAreas', area)}
                        className={styles.checkbox}
                      />
                      <span>{area}</span>
                    </label>
                  ))}
                </div>
              </div>
            </section>

            {/* Hardware Background */}
            <section className={styles.formSection}>
              <h2 className={styles.sectionTitle}>Hardware Background</h2>

              <div className={styles.formGroup}>
                <label htmlFor="hardwareSkillLevel" className={styles.label}>Hardware Skill Level *</label>
                <select
                  id="hardwareSkillLevel"
                  name="hardwareSkillLevel"
                  value={formData.hardwareSkillLevel}
                  onChange={handleInputChange}
                  className={styles.select}
                  required
                >
                  <option value="beginner">Beginner - No hardware experience</option>
                  <option value="intermediate">Intermediate - Some IoT/Robotics projects</option>
                  <option value="advanced">Advanced - Professional hardware development</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    name="hasHardwareExperience"
                    checked={formData.hasHardwareExperience}
                    onChange={handleInputChange}
                    className={styles.checkbox}
                  />
                  <span>I have experience with Hardware / IoT / Robotics</span>
                </label>
              </div>

              {formData.hasHardwareExperience && (
                <div className={styles.formGroup}>
                  <label htmlFor="hardwareDescription" className={styles.label}>
                    Tell us about your hardware experience (optional)
                  </label>
                  <textarea
                    id="hardwareDescription"
                    name="hardwareDescription"
                    value={formData.hardwareDescription}
                    onChange={handleInputChange}
                    className={styles.textarea}
                    rows={3}
                    placeholder="E.g., Built Arduino projects, worked with Raspberry Pi, developed IoT devices..."
                  />
                </div>
              )}
            </section>

            {/* Professional Information */}
            <section className={styles.formSection}>
              <h2 className={styles.sectionTitle}>Professional Information</h2>

              <div className={styles.formGroup}>
                <label htmlFor="professionalRole" className={styles.label}>Professional Role *</label>
                <select
                  id="professionalRole"
                  name="professionalRole"
                  value={formData.professionalRole}
                  onChange={handleInputChange}
                  className={styles.select}
                  required
                >
                  <option value="student">Student - Learning for academic purposes</option>
                  <option value="researcher">Researcher - Conducting research in robotics/AI</option>
                  <option value="engineer">Engineer - Working in industry</option>
                  <option value="hobbyist">Hobbyist - Personal interest and projects</option>
                  <option value="other">Other (please specify)</option>
                </select>
              </div>

              {formData.professionalRole === 'other' && (
                <div className={styles.formGroup}>
                  <label htmlFor="roleOther" className={styles.label}>Specify Your Role *</label>
                  <input
                    type="text"
                    id="roleOther"
                    name="roleOther"
                    value={formData.roleOther}
                    onChange={handleInputChange}
                    className={styles.input}
                    placeholder="E.g., Robotics Consultant, Entrepreneur..."
                    required
                  />
                </div>
              )}

              <div className={styles.formGroup}>
                <label htmlFor="organization" className={styles.label}>Organization (optional)</label>
                <input
                  type="text"
                  id="organization"
                  name="organization"
                  value={formData.organization}
                  onChange={handleInputChange}
                  className={styles.input}
                  placeholder="E.g., MIT, Google, Self-employed..."
                />
              </div>
            </section>

            {/* Error Message */}
            {error && (
              <div className={styles.errorMessage}>
                {error}
              </div>
            )}

            {/* Submit Button */}
            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? 'Creating Account...' : 'Create Account'}
            </button>

            {/* Sign In Link */}
            <p className={styles.authFooter}>
              Already have an account?{' '}
              <Link to="/auth/signin" className={styles.authLink}>
                Sign In
              </Link>
            </p>
          </form>
        </div>
      </div>
    </Layout>
  );
}
