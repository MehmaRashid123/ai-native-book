import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthContext';
import styles from '../components/Auth/styles.module.css';

export default function ProfilePage() {
    const { user, isLoading, authClient } = useAuth();
    const [isEditing, setIsEditing] = useState(false);
    const [formData, setFormData] = useState({
        operating_system: '',
        experience_level: '',
        hardware_access: false
    });
    const [message, setMessage] = useState('');

    useEffect(() => {
        if (user) {
            setFormData({
                operating_system: user.operating_system,
                experience_level: user.experience_level,
                hardware_access: user.hardware_access
            });
        }
    }, [user]);

    if (isLoading) return <Layout title="Profile"><div className="container padding-vert--lg">Loading...</div></Layout>;

    if (!user) {
        return (
            <Layout title="Profile">
                <div className="container padding-vert--lg text--center">
                    <h1>Access Denied</h1>
                    <p>Please <a href="/login">sign in</a> to view your profile.</p>
                </div>
            </Layout>
        );
    }

    const handleLogout = async () => {
        await authClient.signOut();
        window.location.href = '/';
    };

    const handleUpdate = async (e: React.FormEvent) => {
        e.preventDefault();
        setMessage('Updating...');
        
        const { error } = await authClient.updateUser({
            operating_system: formData.operating_system,
            experience_level: formData.experience_level,
            hardware_access: formData.hardware_access
        });

        if (error) {
            setMessage('Error: ' + error.message);
        } else {
            setMessage('Profile updated successfully!');
            setIsEditing(false);
            window.location.reload(); // Refresh to show new data
        }
    };

    return (
        <Layout title="Your Profile">
            <div className="container padding-vert--lg">
                <div className={styles.authForm} style={{margin: '0 auto', maxWidth: '600px'}}>
                    <h2 style={{textAlign: 'left'}}>Your Profile</h2>
                    
                    <div style={{marginBottom: '2rem'}}>
                        <p><strong>Name:</strong> {user.name}</p>
                        <p><strong>Email:</strong> {user.email}</p>
                    </div>

                    <hr className={styles.separator} />
                    <h3>Your Personalization</h3>
                    
                    {message && <div className={styles.error} style={{background: 'rgba(0, 188, 212, 0.1)', color: 'var(--neon-cyan)', borderColor: 'var(--neon-cyan)'}}>{message}</div>}

                    {!isEditing ? (
                        <div style={{marginBottom: '2rem'}}>
                            <p><strong>Operating System:</strong> {user.operating_system}</p>
                            <p><strong>Experience Level:</strong> {user.experience_level}</p>
                            <p><strong>Hardware Access:</strong> {user.hardware_access ? 'Yes' : 'No'}</p>
                            <button onClick={() => setIsEditing(true)} style={{marginTop: '1rem', background: 'transparent', border: '1px solid var(--neon-cyan)', color: 'var(--neon-cyan)'}}>
                                Edit Background
                            </button>
                        </div>
                    ) : (
                        <form onSubmit={handleUpdate}>
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

                            <div style={{display: 'flex', gap: '1rem'}}>
                                <button type="submit">Save Changes</button>
                                <button type="button" onClick={() => setIsEditing(false)} style={{background: 'rgba(255,255,255,0.1)', color: '#fff'}}>Cancel</button>
                            </div>
                        </form>
                    )}

                    <hr className={styles.separator} />
                    <button onClick={handleLogout} style={{background: '#ff4444', marginTop: '1rem'}}>
                        Logout
                    </button>
                </div>
            </div>
        </Layout>
    );
}