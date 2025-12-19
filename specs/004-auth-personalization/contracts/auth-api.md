# API Contracts: Authentication

## Endpoints (Auth Server)

### POST `/api/auth/signup`
Creates a new user and capture personalization data.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123!",
  "name": "John Doe",
  "operating_system": "LINUX",
  "experience_level": "BEGINNER",
  "hardware_access": true
}
```

**Success Response (200)**:
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "operating_system": "LINUX"
  },
  "token": "session_token"
}
```

---

### POST `/api/auth/signin`
Authenticate existing user.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123!"
}
```

**Success Response (200)**:
```json
{
  "session": {
    "token": "session_token",
    "user_id": "uuid"
  }
}
```

---

### GET `/api/auth/get-session`
Retrieve current user session and profile data.

**Headers**:
- `Authorization: Bearer <token>`

**Success Response (200)**:
```json
{
  "user": {
    "id": "uuid",
    "operating_system": "LINUX",
    "experience_level": "BEGINNER"
  }
}
```
