import React from 'react';
import { AuthProvider } from '../components/Auth/AuthContext';

export default function Root({ children }) {
  // Use the live Hugging Face backend for authentication
  const authUrl = "https://mehma-deploy.hf.space";

  return (
    <AuthProvider authUrl={authUrl}>
      {children}
    </AuthProvider>
  );
}