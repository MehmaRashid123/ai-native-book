import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

export default function Signup() {
    const { authClient } = useAuth();
    const [formData, setFormData] = useState({
        email: '',
        password: '',
        name: '',
        operating_system: 'LINUX',
        experience_level: 'BEGINNER',
        hardware_access: false
    });
    const [error, setError] = useState('');
    const [isLoading, setIsLoading] = useState(false);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setIsLoading(true);
        setError('');

        const { data, error } = await authClient.signUp.email({
            email: formData.email,
            password: formData.password,
            name: formData.name,
            operating_system: formData.operating_system,
            experience_level: formData.experience_level,
            hardware_access: formData.hardware_access
        });

        if (error) {
            setError(error.message || 'Signup failed');
        } else {
            window.location.href = '/docs/module-1-ros2/introduction';
        }
        setIsLoading(false);
    };

    return (
        <div className={styles.authContainer}>
            <form className={styles.authForm} onSubmit={handleSubmit}>
                <h2>Create Account</h2>
                {error && <div className={styles.error}>{error}</div>}
                
                <div className={styles.field}>
                    <label>Name</label>
                    <input type="text" required value={formData.name} onChange={e => setFormData({...formData, name: e.target.value})} />
                </div>

                <div className={styles.field}>
                    <label>Email</label>
                    <input type="email" required value={formData.email} onChange={e => setFormData({...formData, email: e.target.value})} />
                </div>

                <div className={styles.field}>
                    <label>Password</label>
                    <input type="password" required value={formData.password} onChange={e => setFormData({...formData, password: e.target.value})} />
                </div>

                <hr className={styles.separator} />
                <h3>Personalize Your Experience</h3>

                <div className={styles.field}>
                    <label>Operating System</label>
                    <select value={formData.operating_system} onChange={e => setFormData({...formData, operating_system: e.target.value})}>
                        <option value="LINUX">Linux (Ubuntu)</option>
                        <option value="WINDOWS">Windows</option>
                        <option value="MAC">macOS</option>
                        <option value="OTHER">Other</option>
                    </select>
                </div>

                <div className={styles.field}>
                    <label>Programming Experience</label>
                    <select value={formData.experience_level} onChange={e => setFormData({...formData, experience_level: e.target.value})}>
                        <option value="BEGINNER">Beginner</option>
                        <option value="INTERMEDIATE">Intermediate</option>
                        <option value="ADVANCED">Advanced</option>
                    </select>
                </div>

                <div className={styles.checkboxField}>
                    <input type="checkbox" id="hardware" checked={formData.hardware_access} onChange={e => setFormData({...formData, hardware_access: e.target.checked})} />
                    <label htmlFor="hardware">I have access to a physical robot</label>
                </div>

                <button type="submit" disabled={isLoading}>
                    {isLoading ? 'Creating Account...' : 'Sign Up'}
                </button>
            </form>
        </div>
    );
}
