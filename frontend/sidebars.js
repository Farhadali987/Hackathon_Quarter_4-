// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Textbook',
      items: [
        'textbook/chapter1-introduction',
        'textbook/chapter2-ros2',
        'textbook/chapter3-gazebo',
        'textbook/chapter3-digital-twin',
        'textbook/chapter4-nvidia-isaac',
        'textbook/chapter5-vla',
        'textbook/chapter6-humanoid-development',
        'textbook/chapter7-conversational-robotics',
        'textbook/chapter8-capstone-hardware',
      ],
      link: {
        type: 'generated-index',
        title: 'Textbook Chapters',
        description: 'Learn about Physical AI and Humanoid Robotics',
        slug: '/textbook',
      },
    },
  ],
  labGuidesSidebar: [
    {
      type: 'category',
      label: 'Lab Guides',
      items: [
        {
          type: 'category',
          label: 'Cloud Labs',
          items: [
            'lab-guides/cloud/lab1-simulation-basics',
          ],
        },
        {
          type: 'category',
          label: 'Hardware Labs',
          items: [
            'lab-guides/hardware/lab1-getting-started',
          ],
        },
      ],
      link: {
        type: 'generated-index',
        title: 'Lab Guides',
        description: 'Practical exercises for Physical AI and Robotics',
        slug: '/labs',
      },
    },
  ],
};

module.exports = sidebars;