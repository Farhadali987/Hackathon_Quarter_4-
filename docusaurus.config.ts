import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the gap between the digital brain and the physical body.',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // IMPORTANT for Vercel
  url: 'https://physical-ai-robotics-textbook.vercel.app',
  baseUrl: '/',

  // GitHub pages deploy config (ignored on Vercel but optional)
  organizationName: 'panaversity',
  projectName: 'physical-ai-robotics-textbook',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/panaversity/physical-ai-robotics-textbook/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: { alt: 'Panaversity Logo', src: 'img/logo.svg' },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        { to: '/auth', label: 'Login/Register', position: 'right' },
        {
          href: 'https://github.com/panaversity/physical-ai-robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
        { type: 'localeDropdown', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [{ label: 'Introduction', to: '/docs/textbook/intro' }],
        },
        {
          title: 'Community',
          items: [
            { label: 'Panaversity', href: 'https://www.panaversity.org/' },
            { label: 'Discord', href: 'https://discordapp.com/invite/docusaurus' },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/panaversity/physical-ai-robotics-textbook',
            },
          ],
        },
      ],
      copyright:
        `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
