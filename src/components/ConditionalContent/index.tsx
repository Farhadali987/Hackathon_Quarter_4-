import React, { ReactNode } from 'react';
import { useUser } from '@site/src/context/UserContext';

type Props = {
  children: ReactNode;
  level: 'beginner' | 'intermediate' | 'advanced';
};

const keywords = {
  beginner: ['arduino', 'raspberry pi'],
  intermediate: ['python', 'c++', 'ros'],
  advanced: ['nvidia jetson', 'robotics', 'vslam'],
};

export default function ConditionalContent({ children, level }: Props): ReactNode {
  const { user } = useUser();

  if (!user) {
    // If no user is logged in, show all content by default
    // or decide on a default behavior
    return <>{children}</>;
  }

  const checkKeywords = (background: string, levelKeywords: string[]): boolean => {
    const backgroundLc = background.toLowerCase();
    return levelKeywords.some(keyword => backgroundLc.includes(keyword));
  };

  const hasSoftwareAccess = checkKeywords(user.softwareBackground, keywords[level]);
  const hasHardwareAccess = checkKeywords(user.hardwareBackground, keywords[level]);

  // A simple logic: show content if either software or hardware background matches the level
  if (hasSoftwareAccess || hasHardwareAccess) {
    return <>{children}</>;
  }

  // A more advanced logic could be implemented to determine if a user has access to a certain level
  // For now, we will just show a placeholder if the user does not have access.

  if (level === 'beginner' && !(hasSoftwareAccess || hasHardwareAccess)) {
    return null;
  }
  if (level === 'intermediate' && !(hasSoftwareAccess || hasHardwareAccess)) {
    return null;
  }
  if (level === 'advanced' && !(hasSoftwareAccess || hasHardwareAccess)) {
    return null;
  }

  return <>{children}</>;
}
