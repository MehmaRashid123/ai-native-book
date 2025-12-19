import { betterAuth } from "better-auth";
import { betterSqlite3Adapter } from "better-auth/adapters/better-sqlite3";
import Database from "better-sqlite3";

const db = new Database(process.env.DATABASE_URL || "sqlite.db");

export const auth = betterAuth({
    database: betterSqlite3Adapter(db, {
        provider: "sqlite",
    }),
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
