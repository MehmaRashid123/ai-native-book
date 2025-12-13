import React from 'react';
import { MantineProvider, createTheme, ColorSchemeScript } from '@mantine/core';
import '@mantine/core/styles.css'; // Import Mantine styles

const sciFiTheme = createTheme({
  fontFamily: 'var(--font-family-base)',
  colors: {
    dark: [
      '#C1C2C5', '#A6A7AB', '#909296', '#5C5F66', '#373A40', '#2C2E33',
      '#25262B', 'var(--space-black)', '#141518', '#101113',
    ],
    neonCyan: [
      '#E3FFFF', '#C7FFFF', '#A8FFFF', '#88FFFF', '#66FFFF', '#44FFFF',
      '#22FFFF', 'var(--neon-cyan)', '#00BBD4', '#00AABB',
    ],
    sciFiViolet: [
      '#F4E3FF', '#E3C7FF', '#D1A8FF', '#BF88FF', '#AB66FF', '#9744FF',
      '#8322FF', 'var(--violet)', '#6F00D4', '#5B00BB',
    ],
  },
  primaryColor: 'neonCyan',
});

export default function Root({ children }) {
  return (
    <MantineProvider theme={sciFiTheme} defaultColorScheme="dark">
      <ColorSchemeScript defaultColorScheme="dark" />
      {children}
    </MantineProvider>
  );
}
