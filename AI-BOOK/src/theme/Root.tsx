import React from 'react';
import { AuthProvider } from '../components/Auth/AuthContext';

export default function Root({ children }) {
  // Base URL of the auth server
  const authUrl = "http://localhost:3001";

  return (
    <AuthProvider authUrl={authUrl}>
      {children}
    </AuthProvider>
  );
}