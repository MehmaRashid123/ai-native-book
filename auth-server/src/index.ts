import { Hono } from "hono";
import { cors } from "hono/cors";
import { auth } from "./auth";

const app = new Hono();

app.use("*", cors({
    origin: process.env.frontend_url || "http://localhost:3000",
    allowHeaders: ["Content-Type", "Authorization"],
    allowMethods: ["POST", "GET", "OPTIONS"],
    exposeHeaders: ["Content-Length"],
    maxAge: 600,
    credentials: true,
}));

app.on(["POST", "GET"], "/api/auth/*", (c) => {
    return auth.handler(c.req.raw);
});

const port = Number(process.env.PORT) || 3001;

console.log(`Auth server is running on port ${port}`);

export default {
    port,
    fetch: app.fetch,
};
