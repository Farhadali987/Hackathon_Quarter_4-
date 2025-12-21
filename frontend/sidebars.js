// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['textbook/chapter1-introduction'],
    },
    {
      type: 'category',
      label: 'ROS 2',
      items: ['textbook/chapter2-ros2'],
    },
    {
      type: 'category',
      label: 'Digital Twin',
      items: ['textbook/chapter3-digital-twin'],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: ['textbook/chapter4-nvidia-isaac'],
    },
    {
      type: 'category',
      label: 'VLA',
      items: ['textbook/chapter5-vla'],
    },
    {
      type: 'category',
      label: 'Humanoid Development',
      items: ['textbook/chapter6-humanoid-development'],
    },
    {
      type: 'category',
      label: 'Conversational Robotics',
      items: ['textbook/chapter7-conversational-robotics'],
    },
    {
      type: 'category',
      label: 'Capstone Hardware Project',
      items: ['textbook/chapter8-capstone-hardware'],
    },
    {
      type: 'category',
      label: 'Lab Guides',
      items: [
        'lab-guides/hardware/lab1-getting-started',
        'lab-guides/cloud/lab1-simulation-basics'
      ],
    },
  ],
};

module.exports = sidebars;