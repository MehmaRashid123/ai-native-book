import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const TranslationButton = () => {
  return (
    <div className={clsx(styles.translationButton, 'button button--primary')}>
      Translate to Urdu (Placeholder)
    </div>
  );
};

export default TranslationButton;
