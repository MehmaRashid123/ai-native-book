import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthContext';
import styles from '../components/Auth/styles.module.css';

export default function ProfilePage() {
    const { user, isLoading, authClient } = useAuth();

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
                    
                    <div style={{marginBottom: '2rem'}}>
                        <p><strong>Operating System:</strong> {user.operating_system}</p>
                        <p><strong>Experience Level:</strong> {user.experience_level}</p>
                        <p><strong>Hardware Access:</strong> {user.hardware_access ? 'Yes' : 'No'}</p>
                    </div>

                    <button onClick={handleLogout} className={styles.buttonSecondary} style={{background: '#ff4444'}}>
                        Logout
                    </button>
                </div>
            </div>
        </Layout>
    );
}
