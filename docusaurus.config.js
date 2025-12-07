// @ts-check
const {themes} = require('prism-react-renderer');
const lightTheme = themes.github;
const darkTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Humanoid Robotics',
  tagline: 'Master the future of intelligent robotics through hands-on learning in ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action systems',
  favicon: 'img/favicon.ico',

  url: 'https://your-username.github.io',
  baseUrl: '/hackathon-Physical-AI-Humanoid-Robotics-2/',

  organizationName: 'your-username',
  projectName: 'hackathon-Physical-AI-Humanoid-Robotics-2',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: 'docs',
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/your-username/hackathon-Physical-AI-Humanoid-Robotics-2/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.jpg',
      navbar: {
        title: 'Physical AI Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/your-username/hackathon-Physical-AI-Humanoid-Robotics-2',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Modules',
            items: [
              {
                label: 'Introduction',
                to: '/docs/introduction/what-is-physical-ai',
              },
              {
                label: 'ROS 2',
                to: '/docs/ros2/introduction-to-ros2',
              },
              {
                label: 'Digital Twin',
                to: '/docs/digital-twin/digital-twin-systems',
              },
              {
                label: 'NVIDIA Isaac',
                to: '/docs/isaac/isaac-platform-intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Setup Guides',
                to: '/docs/setup-guides/digital-twin-workstation',
              },
              {
                label: 'Glossary',
                to: '/docs/references/robotics-glossary',
              },
              {
                label: 'Downloads',
                to: '/docs/references/downloads-resources',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/hackathon-Physical-AI-Humanoid-Robotics-2',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} GIAIC. Built with Docusaurus.`,
      },
      prism: {
        theme: lightTheme,
        darkTheme: darkTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'cpp'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
    }),
};

module.exports = config;
