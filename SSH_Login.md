# SSH Login Setup

## Initial Setup
1. Copy the environment template:
   ```bash
   cp .env.example .env
   ```

2. Edit `.env` with your actual credentials:
   ```bash
   # Update ROVER_USER and ROVER_PASSWORD with real values
   ROVER_USER=your_rover_username  
   ROVER_PASSWORD=your_secure_password
   ```

3. Connect to rover:
   ```bash
   ssh $ROVER_USER@rover-ip-address
   ```

> [!IMPORTANT]
> Never commit the `.env` file - it contains sensitive credentials.