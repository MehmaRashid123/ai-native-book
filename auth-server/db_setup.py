import psycopg2
import os

# Manual Database Setup Script for BetterAuth - FIXED for PostgreSQL naming
DATABASE_URL = "postgresql://neondb_owner:npg_X2GaZbPFj1LN@ep-autumn-night-a4722g1t-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require"

sql_commands = [
    # Drop existing tables to ensure a clean slate with correct naming
    "DROP TABLE IF EXISTS verification CASCADE;",
    "DROP TABLE IF EXISTS account CASCADE;",
    "DROP TABLE IF EXISTS session CASCADE;",
    "DROP TABLE IF EXISTS \"user\" CASCADE;",
    
    """
    CREATE TABLE \"user\" (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        email TEXT NOT NULL UNIQUE,
        \"emailVerified\" BOOLEAN NOT NULL,
        image TEXT,
        \"createdAt\" TIMESTAMP NOT NULL,
        \"updatedAt\" TIMESTAMP NOT NULL,
        operating_system TEXT NOT NULL DEFAULT 'OTHER',
        experience_level TEXT NOT NULL DEFAULT 'BEGINNER',
        hardware_access BOOLEAN NOT NULL DEFAULT FALSE
    );
    """,
    """
    CREATE TABLE session (
        id TEXT PRIMARY KEY,
        \"expiresAt\" TIMESTAMP NOT NULL,
        token TEXT NOT NULL UNIQUE,
        \"createdAt\" TIMESTAMP NOT NULL,
        \"updatedAt\" TIMESTAMP NOT NULL,
        \"ipAddress\" TEXT,
        \"userAgent\" TEXT,
        \"userId\" TEXT NOT NULL REFERENCES \"user\"(id) ON DELETE CASCADE
    );
    """,
    """
    CREATE TABLE account (
        id TEXT PRIMARY KEY,
        \"accountId\" TEXT NOT NULL,
        \"providerId\" TEXT NOT NULL,
        \"userId\" TEXT NOT NULL REFERENCES \"user\"(id) ON DELETE CASCADE,
        \"accessToken\" TEXT,
        \"refreshToken\" TEXT,
        \"idToken\" TEXT,
        \"accessTokenExpiresAt\" TIMESTAMP,
        \"refreshTokenExpiresAt\" TIMESTAMP,
        scope TEXT,
        password TEXT,
        \"createdAt\" TIMESTAMP NOT NULL,
        \"updatedAt\" TIMESTAMP NOT NULL
    );
    """,
    """
    CREATE TABLE verification (
        id TEXT PRIMARY KEY,
        identifier TEXT NOT NULL,
        value TEXT NOT NULL,
        \"expiresAt\" TIMESTAMP NOT NULL,
        \"createdAt\" TIMESTAMP,
        \"updatedAt\" TIMESTAMP
    );
    """
]

def setup_db():
    print("Connecting to Neon PostgreSQL...")
    try:
        conn = psycopg2.connect(DATABASE_URL)
        cur = conn.cursor()
        
        for command in sql_commands:
            print(f"Executing command...")
            cur.execute(command)
        
        conn.commit()
        cur.close()
        conn.close()
        print("\n✅ Database RE-INITIALIZED successfully with correct column names!")
    except Exception as e:
        print(f"\n❌ Error setting up database: {e}")

if __name__ == "__main__":
    setup_db()