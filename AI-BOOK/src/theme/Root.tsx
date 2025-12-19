import React from 'react';
import { AuthProvider } from '../components/Auth/AuthContext';

export default function Root({ children }) {
  // Use the local auth server by default
  const authUrl = "http://localhost:3001";

  return (
    <AuthProvider authUrl={authUrl}>
      {children}
    </AuthProvider>
  );
}