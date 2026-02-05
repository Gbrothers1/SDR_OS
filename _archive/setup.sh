#!/bin/bash

# Install dependencies
echo "Installing dependencies..."
npm install

# Create necessary directories if they don't exist
echo "Creating necessary directories..."
mkdir -p src/client/components
mkdir -p dist

# Build the application
echo "Building the application..."
npm run build

# Start the server
echo "Starting the server..."
npm start 