import React from 'react';

interface PersonalizedProps {
  showFor: 'beginner' | 'advanced' | 'hardware';
  children: React.ReactNode;
}

interface UserProfile {
  software_skill: string; // e.g., "Beginner", "Intermediate", "Advanced"
  hardware_access: boolean; // e.g., true, false
}

const Personalized: React.FC<PersonalizedProps> = ({ showFor, children }) => {
  // Try to retrieve user from localStorage
  const userJson = typeof window !== 'undefined' ? localStorage.getItem('user') : null;
  const user: UserProfile | null = userJson ? JSON.parse(userJson) : null;

  if (!user) {
    // If no user is logged in, or user data is not available,
    // we should render based on a default assumption or return null.
    // For now, let's assume if no user, then don't show personalized content.
    return null; 
  }

  const userSoftwareSkill = user.software_skill.toLowerCase();

  switch (showFor) {
    case 'beginner':
      if (userSoftwareSkill === 'beginner') {
        return <>{children}</>;
      }
      break;
    case 'advanced':
      if (userSoftwareSkill === 'advanced') {
        return <>{children}</>;
      }
      break;
    case 'hardware':
      if (user.hardware_access === true) {
        return <>{children}</>;
      }
      break;
    default:
      return null;
  }

  return null;
};

export default Personalized;
