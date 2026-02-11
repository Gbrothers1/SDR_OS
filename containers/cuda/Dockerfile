FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        build-essential \
        python3 \
        python3-venv \
        python3-pip \
        libegl1 \
        libgl1 \
        libglib2.0-0 \
        libxrender1 \
        libopengl0 \
        libturbojpeg0-dev \
    && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && corepack enable \
    && rm -rf /var/lib/apt/lists/*

ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

RUN pip install --no-cache-dir uv

WORKDIR /workspace

# Use lockfiles for reproducible installs.
# Copy only lockfiles first to maximize layer cache.
COPY pyproject.toml uv.lock ./
COPY package.json package-lock.json* pnpm-lock.yaml* ./

# Install Python packages into /opt/venv so host volume mount doesn't shadow them.
# uv sync creates .venv which gets overwritten by the host mount, so we sync then
# copy the result into /opt/venv and clean up.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --group cuda \
    && cp -a /workspace/.venv/lib/python3.10/site-packages/* /opt/venv/lib/python3.10/site-packages/ \
    && rm -rf /workspace/.venv

RUN --mount=type=cache,target=/root/.npm \
    --mount=type=cache,target=/root/.local/share/pnpm/store \
    if [ -f pnpm-lock.yaml ]; then \
      corepack enable && pnpm install --frozen-lockfile; \
    elif [ -f package-lock.json ]; then \
      npm ci; \
    fi

COPY . /workspace

ENV PYOPENGL_PLATFORM=egl

CMD ["sleep", "infinity"]
