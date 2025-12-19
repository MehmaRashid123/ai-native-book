# Data Model: Authentication & User Profiles

## Entities

### User (BetterAuth Extended)
The primary identity record, including custom personalization fields.

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Primary key. |
| `email` | String | Unique user email. |
| `password` | String | Hashed password. |
| `name` | String | Full name. |
| `operating_system` | Enum | `LINUX`, `WINDOWS`, `MAC`, `OTHER`. |
| `experience_level` | Enum | `BEGINNER`, `INTERMEDIATE`, `ADVANCED`. |
| `hardware_access` | Boolean | True if user has access to a physical robot. |
| `created_at` | DateTime | Timestamp of creation. |

### Session
Managed by BetterAuth.

| Field | Type | Description |
|-------|------|-------------|
| `id` | String | Session ID. |
| `user_id` | UUID | Foreign key to User. |
| `expires_at` | DateTime | Expiry timestamp. |
| `ip_address` | String | (Optional) Tracking for security. |

## Relationships
- **User** 1:N **Session**: A user can have multiple active sessions across devices.

## Validation Rules
- `email`: Must be a valid Panaversity or standard email format.
- `password`: Minimum 8 characters, must include at least one number and one symbol.
- `operating_system`: Mandatory during signup to enable personalization.
