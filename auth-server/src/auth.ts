import { betterAuth } from "better-auth";
import { Pool } from "pg";

const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
});

export const auth = betterAuth({
    database: pool,
    // CRITICAL for Vercel + Hugging Face integration:
    trustedOrigins: ["http://localhost:3000", "https://ai-native-book-psi.vercel.app"],
    baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
    
    // Cookie configuration for cross-domain sessions
    advanced: {
        cookiePrefix: "ai-book",
        useCookieCache: true,
    },
    
    session: {
        cookie: {
            sameSite: "none",
            secure: true,
        }
    },

    emailAndPassword: {
        enabled: true,
    },
    user: {
        additionalFields: {
            operating_system: {
                type: "string",
                required: true,
                defaultValue: "OTHER",
            },
            experience_level: {
                type: "string",
                required: true,
                defaultValue: "BEGINNER",
            },
            hardware_access: {
                type: "boolean",
                required: true,
                defaultValue: false,
            }
        }
    }
});