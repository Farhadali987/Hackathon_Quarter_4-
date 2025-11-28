import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    'textbook/intro', // The introduction file
    {
      type: 'category',
      label: 'Module 1: Foundations',
      link: {
        type: 'generated-index',
        title: 'Module 1: Foundations',
        description: 'Understand the core principles of Physical AI and the Robotic Operating System.',
        slug: '/category/module-1-foundations',
      },
      items: [
        'textbook/chapter1-introduction',
        'textbook/chapter2-ros2',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation & Digital Twins',
      link: {
        type: 'generated-index',
        title: 'Module 2: Simulation & Digital Twins',
        description: 'Explore the creation and use of digital twins for robot development and testing.',
        slug: '/category/module-2-simulation-digital-twins',
      },
      items: [
        'textbook/chapter3-digital-twin',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced AI & Perception',
      link: {
        type: 'generated-index',
        title: 'Module 3: Advanced AI & Perception',
        description: 'Dive into advanced AI concepts and hardware acceleration for robot perception.',
        slug: '/category/module-3-advanced-ai-perception',
      },
      items: [
        'textbook/chapter4-nvidia-isaac',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Interaction & Autonomy',
      link: {
        type: 'generated-index',
        title: 'Module 4: Interaction & Autonomy',
        description: 'Learn about Vision-Language-Action models, humanoid movement, and conversational capabilities.',
        slug: '/category/module-4-interaction-autonomy',
      },
      items: [
        'textbook/chapter5-vla',
        'textbook/chapter6-humanoid-development',
        'textbook/chapter7-conversational-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Capstone & Resources',
      link: {
        type: 'generated-index',
        title: 'Capstone Project & Resources',
        description: 'Prepare for the final project and understand the necessary hardware and infrastructure.',
        slug: '/category/capstone-resources',
      },
      items: [
        'textbook/chapter8-capstone-hardware',
      ],
    },
  ],
};

export default sidebars;
