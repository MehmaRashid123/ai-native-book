import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { createAuthClient } from "better-auth/react";

interface User {
    id: string;
    email: string;
    name: string;
    operating_system: string;
    experience_level: string;
    hardware_access: boolean;
}

interface AuthContextType {
    user: User | null;
    session: any | null;
    isLoading: boolean;
    authClient: any;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider = ({ children, authUrl }: { children: ReactNode, authUrl: string }) => {
    const authClient = createAuthClient({
        baseURL: authUrl,
        fetchOptions: {
            authPath: "/api/auth",
            // This is required to send cookies to the Hugging Face domain
            credentials: "include" 
        }
    });

    const [user, setUser] = useState<User | null>(null);
    const [session, setSession] = useState<any | null>(null);
    const [isLoading, setIsLoading] = useState(true);

    useEffect(() => {
        const fetchSession = async () => {
            setIsLoading(true);
            const { data, error } = await authClient.getSession();
            if (data) {
                setSession(data.session);
                setUser(data.user as unknown as User);
            }
            setIsLoading(false);
        };
        fetchSession();
    }, []);

    return (
        <AuthContext.Provider value={{ user, session, isLoading, authClient }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => {
    const context = useContext(AuthContext);
    if (context === undefined) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
};
