FROM python:3.11-slim

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        build-essential \
        nodejs \
        npm \
    && rm -rf /var/lib/apt/lists/*

ENV VIRTUAL_ENV=/opt/venv
RUN python -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

RUN pip install --no-cache-dir uv

WORKDIR /workspace

ARG MLX_VARIANT=mlx_cpu

COPY pyproject.toml uv.lock ./
COPY requirements/requirements-mlx.txt ./requirements/requirements-mlx.txt
COPY requirements/requirements-mlx-cpu.txt ./requirements/requirements-mlx-cpu.txt
COPY requirements/requirements-mlx-cuda12.txt ./requirements/requirements-mlx-cuda12.txt
COPY requirements/requirements-mlx-cuda13.txt ./requirements/requirements-mlx-cuda13.txt
COPY package.json package-lock.json pnpm-lock.yaml ./

RUN if [ -f uv.lock ]; then uv sync --frozen --group ${MLX_VARIANT}; fi \
    && if [ -f requirements/requirements-${MLX_VARIANT}.txt ]; then uv pip sync requirements/requirements-${MLX_VARIANT}.txt; fi \
    && if [ -f package-lock.json ]; then npm ci; fi \
    && if [ -f pnpm-lock.yaml ]; then corepack enable && pnpm install --frozen-lockfile; fi

COPY . /workspace

CMD ["bash"]
