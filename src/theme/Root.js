import React from 'react';
import TranslationWrapper from '../components/TranslationWrapper';

export default function Root({ children }) {
  return <TranslationWrapper>{children}</TranslationWrapper>;
}