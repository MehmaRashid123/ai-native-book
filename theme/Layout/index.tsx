import React from 'react';
import Layout from '@theme-original/Layout';
import { AuthProvider } from '@site/src/context/AuthContext';
import NavbarLoginBtn from '@site/src/components/NavbarLoginBtn';
import ChatWidget from '@site/src/components/ChatWidget';

export default function LayoutWrapper(props: any) {
  return (
    <AuthProvider>
      <Layout {...props} />
      <NavbarLoginBtn />
      <ChatWidget />
    </AuthProvider>
  );
}