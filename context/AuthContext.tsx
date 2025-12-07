import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';
import AuthModal from '../components/AuthModal';

interface User {
  id: number;
  email: string;
  software_skill: string;
  hardware_access: boolean;
}

interface AuthContextType {
  user: User | null;
  token: string | null;
  loading: boolean;
  login: (token: string, user: User) => void;
  logout: () => void;
  openAuthModal: (isLoginMode?: boolean) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Hardcoded API Base URL as requested
const API_BASE_URL = 'http://localhost:8000';

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [authModalLoginMode, setAuthModalLoginMode] = useState(true);

  // Initialize state from local storage
  useEffect(() => {
    const initializeAuth = async () => {
      const storedToken = localStorage.getItem('access_token');
      const storedUser = localStorage.getItem('user');

      if (storedToken && storedUser) {
        setToken(storedToken);
        // Optimistically set user from local storage
        setUser(JSON.parse(storedUser));
        
        // Verify token with backend
        try {
          const response = await fetch(`${API_BASE_URL}/api/users/me`, {
            headers: {
              'Authorization': `Bearer ${storedToken}`,
            },
          });
          
          if (response.ok) {
            const verifiedUser = await response.json();
            setUser(verifiedUser);
            localStorage.setItem('user', JSON.stringify(verifiedUser));
          } else {
            console.warn('Token invalid, logging out.');
            handleLogout();
          }
        } catch (error) {
          console.error('Network error during token verification:', error);
          // Don't logout immediately on network error to allow offline usage if needed, 
          // or logout if strict security is required. For now, we'll keep the local state 
          // but maybe set a "warning" flag if we had one.
          // If strict: handleLogout();
        }
      }
      setLoading(false);
    };

    initializeAuth();
  }, []);

  const handleLogin = (newToken: string, newUser: User) => {
    localStorage.setItem('access_token', newToken);
    localStorage.setItem('user', JSON.stringify(newUser));
    setToken(newToken);
    setUser(newUser);
    setIsAuthModalOpen(false);
  };

  const handleLogout = () => {
    localStorage.removeItem('access_token');
    localStorage.removeItem('user');
    setToken(null);
    setUser(null);
  };

  const openAuthModal = (isLoginMode: boolean = true) => {
    setAuthModalLoginMode(isLoginMode);
    setIsAuthModalOpen(true);
  };

  return (
    <AuthContext.Provider value={{
      user,
      token,
      loading,
      login: handleLogin,
      logout: handleLogout,
      openAuthModal
    }}>
      {children}
      <AuthModal 
        isOpen={isAuthModalOpen} 
        onClose={() => setIsAuthModalOpen(false)} 
        onSuccess={handleLogin}
        isLoginMode={authModalLoginMode}
      />
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};