import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master Embodied Intelligence with ROS 2, Isaac Sim, and VLA',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://panaversity.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'panaversity', // Usually your GitHub org/user name.
  projectName: 'ai-native-book', // Usually your repo name.

  onBrokenLinks: 'warn', // Changed from 'throw' to 'warn'
  // onBrokenMarkdownLinks: 'warn', // This will be handled by the top-level onBrokenLinks

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: { label: 'English', direction: 'ltr' },
      ur: { label: 'Urdu', direction: 'rtl' },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Removed incorrect markdown configuration from here
        },
        blog: false, // Disable the blog plugin
        theme: {
          customCss: './css/custom.css', // Corrected path
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      style: 'dark',
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'My Site Logo',
        src: 'img/docusaurus.png', // Corrected logo src
      },
      items: [
        {
          type: 'doc',
          docId: 'intro/physical-ai',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'localeDropdown', // Locale dropdown for i18n
          position: 'right',
        },
        // Removed custom-NavbarLoginBtn from here
        {
          href: 'https://github.com/panaversity/ai-native-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'X',
              href: 'https://x.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/panaversity/ai-native-book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaverse. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: true,
      respectPrefersColorScheme: false,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
