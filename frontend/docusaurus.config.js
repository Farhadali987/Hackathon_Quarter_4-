import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'AI Textbook for Physical AI & Humanoid Robotics',
  tagline: 'An interactive, AI-native textbook for learning Physical AI and Humanoid Robotics concepts',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-deployment-url.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Panaversity', // Usually your GitHub org/user name.
  projectName: 'ai-textbook-physical-ai', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/panaversity/ai-textbook-physical-ai/tree/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'AI Textbook: Physical AI & Robotics',
      logo: {
        alt: 'AI Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'docSidebar',
          sidebarId: 'labGuidesSidebar',
          position: 'left',
          label: 'Lab Guides',
        },
        { to: '/chat', label: 'AI Chatbot', position: 'right' },
        { to: '/auth', label: 'Login', position: 'right' },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/panaversity/ai-textbook-physical-ai',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/textbook/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Panaversity',
              href: 'https://www.panaversity.org/',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/panaversity/ai-textbook-physical-ai',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity Hackathon I. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;