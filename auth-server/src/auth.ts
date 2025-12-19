import { betterAuth } from "better-auth";
import { Pool } from "pg";

const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
});

export const auth = betterAuth({
    database: pool,
    // Add this to fix the 403 Forbidden / CSRF issue
    trustedOrigins: ["http://localhost:3000", "http://127.0.0.1:3000"],
    baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
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
