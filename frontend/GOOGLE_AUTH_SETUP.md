# Google Authentication Setup

This guide will help you set up real Google authentication for the ZeroPanic app.

## Step 1: Create Google Cloud Project

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Name your project (e.g., "ZeroPanic")

## Step 2: Enable Google Sign-In API

1. In the Google Cloud Console, go to "APIs & Services" > "Library"
2. Search for "Google Sign-In API" or "Google+ API"
3. Click on it and enable it

## Step 3: Create OAuth 2.0 Credentials

1. Go to "APIs & Services" > "Credentials"
2. Click "Create Credentials" > "OAuth 2.0 Client IDs"
3. Choose "Web application" as the application type
4. Add authorized origins:
   - `http://localhost:3000` (for local development)
   - `https://your-domain.com` (for production)
5. Add authorized redirect URIs:
   - `http://localhost:3000` (for local development)
   - `https://your-domain.com` (for production)
6. Click "Create"
7. Copy the Client ID

## Step 4: Configure the App

1. Open `frontend/web/index.html`
2. Replace the placeholder client ID:
   ```html
   <meta name="google-signin-client_id" content="YOUR_ACTUAL_CLIENT_ID.apps.googleusercontent.com">
   ```

## Step 5: Test the Authentication

1. Run the Flutter app: `flutter run -d chrome`
2. Click "Sign in with Google"
3. You should see the real Google OAuth flow
4. After signing in, you'll see your actual Gmail address

## Troubleshooting

- **"ClientID not set" error**: Make sure the client ID is correctly set in `web/index.html`
- **"This app isn't verified" warning**: This is normal for development. Click "Advanced" > "Go to [app name] (unsafe)"
- **CORS errors**: Make sure your domain is added to authorized origins in Google Cloud Console

## Production Deployment

For production, make sure to:
1. Add your production domain to authorized origins
2. Update the client ID in your production build
3. Consider setting up proper domain verification in Google Cloud Console
