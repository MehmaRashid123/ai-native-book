import React from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import useIsBrowser from '@docusaurus/useIsBrowser';

const NavbarLoginBtn: React.FC = () => {
  const { user, loading, logout, openAuthModal } = useAuth();
  const isBrowser = useIsBrowser();

  if (!isBrowser) {
    return null;
  }

  // Styles as per strict requirements
  const containerStyle: React.CSSProperties = {
    position: 'fixed',
    top: '15px',
    right: '80px',
    zIndex: 9999999,
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
  };

  const buttonStyle: React.CSSProperties = {
    backgroundColor: '#2563eb', // Blue
    color: 'white',
    border: '1px solid white',
    padding: '6px 16px',
    borderRadius: '20px',
    cursor: 'pointer',
    fontWeight: 'bold',
    fontSize: '14px',
    boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
  };

  if (loading) {
    return (
      <div style={containerStyle}>
        <span style={{ 
          ...buttonStyle, 
          backgroundColor: '#6b7280', // Grey for loading
          cursor: 'default' 
        }}>
          Loading...
        </span>
      </div>
    );
  }

  return (
    <div style={containerStyle}>
      {user ? (
        <>
          <span style={{ 
            color: 'white', 
            fontWeight: 'bold', 
            textShadow: '0 1px 2px rgba(0,0,0,0.8)',
            marginRight: '5px' 
          }}>
            {user.email}
          </span>
          <button
            onClick={logout}
            style={{ 
              ...buttonStyle, 
              backgroundColor: '#dc2626', // Red for logout
              borderColor: '#fee2e2' 
            }}
          >
            Logout
          </button>
        </>
      ) : (
        <button
          onClick={() => openAuthModal(true)}
          style={buttonStyle}
        >
          Login / Signup
        </button>
      )}
    </div>
  );
};

export default NavbarLoginBtn;