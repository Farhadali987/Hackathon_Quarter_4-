import React, { createContext, useState, useContext, ReactNode } from 'react';

type UserProfile = {
  email: string;
  softwareBackground: string;
  hardwareBackground: string;
};

type UserContextType = {
  user: UserProfile | null;
  login: (profile: UserProfile) => void;
  logout: () => void;
};

const UserContext = createContext<UserContextType | undefined>(undefined);

export function UserProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserProfile | null>(null);

  const login = (profile: UserProfile) => {
    setUser(profile);
  };

  const logout = () => {
    setUser(null);
  };

  return (
    <UserContext.Provider value={{ user, login, logout }}>
      {children}
    </UserContext.Provider>
  );
}

export function useUser() {
  const context = useContext(UserContext);
  if (context === undefined) {
    throw new Error('useUser must be used within a UserProvider');
  }
  return context;
}
