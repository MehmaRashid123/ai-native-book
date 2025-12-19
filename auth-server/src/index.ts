import { Hono } from "hono";
import { cors } from "hono/cors";
import { serve } from "@hono/node-server";
import { auth } from "./auth";

const app = new Hono();

// Enable CORS for all routes
app.use("*", cors({
    origin: (origin) => {
        const allowedOrigins = [
            "http://localhost:3000", 
            "http://127.0.0.1:3000", 
            "https://ai-native-book-psi.vercel.app"
        ];
        if (allowedOrigins.includes(origin || "") || !origin) {
            return origin;
        }
        return "https://ai-native-book-psi.vercel.app"; // Fallback to production
    },
    allowHeaders: ["Content-Type", "Authorization"],
    allowMethods: ["POST", "GET", "OPTIONS"],
    exposeHeaders: ["Content-Length", "Set-Cookie"],
    maxAge: 600,
    credentials: true,
}));

// BetterAuth handler
app.all("/api/auth/*", (c) => {
    return auth.handler(c.req.raw);
});

// Error handler to catch and log 500 errors
app.onError((err, c) => {
  console.error("SERVER ERROR:", err);
  return c.text("Internal Server Error", 500);
});

const port = Number(process.env.PORT) || 3001;

console.log(`Auth server is starting on port ${port}...`);

serve({
  fetch: app.fetch,
  port
});
