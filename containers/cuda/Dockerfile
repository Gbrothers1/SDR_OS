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
        nodejs \
        npm \
    && rm -rf /var/lib/apt/lists/*

ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

RUN pip install --no-cache-dir uv

WORKDIR /workspace

# Use lockfiles for reproducible installs.
# Copy only lockfiles first to maximize layer cache.
COPY pyproject.toml uv.lock ./
COPY requirements/requirements-cuda.txt ./requirements/requirements-cuda.txt
COPY package.json package-lock.json pnpm-lock.yaml ./

RUN if [ -f uv.lock ]; then uv sync --frozen --group cuda; fi \
    && if [ -f requirements/requirements-cuda.txt ]; then uv pip sync requirements/requirements-cuda.txt; fi \
    && if [ -f package-lock.json ]; then npm ci; fi \
    && if [ -f pnpm-lock.yaml ]; then corepack enable && pnpm install --frozen-lockfile; fi

COPY . /workspace

CMD ["bash"]
