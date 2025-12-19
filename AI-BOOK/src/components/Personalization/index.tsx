import React, { ReactNode } from 'react';
import { useAuth } from '../Auth/AuthContext';
import styles from './styles.module.css';

interface PersonalizedContentProps {
    children: ReactNode;
    os?: 'LINUX' | 'WINDOWS' | 'MAC' | 'OTHER';
    level?: 'BEGINNER' | 'INTERMEDIATE' | 'ADVANCED';
    hasHardware?: boolean;
    fallback?: ReactNode;
}

export default function PersonalizedContent({ 
    children, 
    os, 
    level, 
    hasHardware,
    fallback = null 
}: PersonalizedContentProps) {
    const { user, isLoading } = useAuth();

    if (isLoading) return null;

    // If no user is logged in, show fallback (or nothing)
    if (!user) return <>{fallback}</>;

    // Check conditions
    const osMatch = !os || user.operating_system === os;
    const levelMatch = !level || user.experience_level === level;
    const hardwareMatch = hasHardware === undefined || user.hardware_access === hasHardware;

    if (osMatch && levelMatch && hardwareMatch) {
        return (
            <div className={styles.personalizedBlock}>
                <div className={styles.badge}>Tailored for you</div>
                {children}
            </div>
        );
    }

    return <>{fallback}</>;
}
