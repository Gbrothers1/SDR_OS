# SDR_OS Node.js server — Socket.io gamepad relay + API
# Caddy serves static assets; this container only handles dynamic routes.

FROM node:20-alpine

WORKDIR /app

# Install production dependencies only
COPY package.json package-lock.json ./
RUN --mount=type=cache,target=/root/.npm \
    npm ci --omit=dev

# Copy server entrypoint
COPY server.js ./

EXPOSE 3000

CMD ["node", "server.js"]
