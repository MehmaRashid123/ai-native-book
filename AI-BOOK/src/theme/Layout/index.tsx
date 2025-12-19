/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import React from 'react';
import clsx from 'clsx';
import {PageMetadata, ThemeClassNames} from '@docusaurus/theme-common';
import {useKeyboardNavigation} from '@docusaurus/theme-common/internal';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import SkipToContent from '@theme/SkipToContent';
import AnnouncementBar from '@theme/AnnouncementBar';
import BackToTopButton from '@theme/BackToTopButton';
import FloatingChatbotButton from '@site/src/components/FloatingChatbotButton'; // New import
import { useAuth } from '@site/src/components/Auth/AuthContext';
import styles from './styles.module.css';
export default function Layout(props) {
  const { user } = useAuth();
  const {
    children,
    noFooter,
    wrapperClassName,
    // TODO: move to metadata if `title` and `description` are only used for PageMetadata
    title,
    description,
  } = props;
  useKeyboardNavigation();
  return (
    <LayoutProvider>
      <PageMetadata title={title} description={description} />

      <SkipToContent />

      <AnnouncementBar />

      <Navbar />

      <div
        id={ThemeClassNames.wrapper.main}
        className={clsx('main-wrapper', wrapperClassName)}>
        <ErrorBoundary fallback={(error, info) => <ErrorPage error={error} info={info} />}>
          {children}
        </ErrorBoundary>
      </div>

      {!noFooter && <Footer />}
      <FloatingChatbotButton />
      <BackToTopButton />
    </LayoutProvider>
  );
}

function ErrorPage({error, info}) {
  return (
    <div>
      <h1>Error</h1>
      <p>{error.message}</p>
      <pre>{info.componentStack}</pre>
    </div>
  );
}
