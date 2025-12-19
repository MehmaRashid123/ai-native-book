import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

export default function Login() {
    const { authClient } = useAuth();
    const [formData, setFormData] = useState({
        email: '',
        password: ''
    });
    const [error, setError] = useState('');
    const [isLoading, setIsLoading] = useState(false);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setIsLoading(true);
        setError('');

        const { data, error } = await authClient.signIn.email({
            email: formData.email,
            password: formData.password
        });

        if (error) {
            setError(error.message || 'Login failed');
        } else {
            window.location.href = '/docs/module-1-ros2/introduction';
        }
        setIsLoading(false);
    };

    return (
        <div className={styles.authContainer}>
            <form className={styles.authForm} onSubmit={handleSubmit}>
                <h2>Sign In</h2>
                {error && <div className={styles.error}>{error}</div>}
                
                <div className={styles.field}>
                    <label>Email</label>
                    <input type="email" required value={formData.email} onChange={e => setFormData({...formData, email: e.target.value})} />
                </div>

                <div className={styles.field}>
                    <label>Password</label>
                    <input type="password" required value={formData.password} onChange={e => setFormData({...formData, password: e.target.value})} />
                </div>

                <button type="submit" disabled={isLoading}>
                    {isLoading ? 'Signing In...' : 'Login'}
                </button>
                
                <div className={styles.footer}>
                    Don't have an account? <a href="/signup">Sign up</a>
                </div>
            </form>
        </div>
    );
}
