FROM python:3.11-slim

# Install system dependencies including Node.js
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    build-essential \
    && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install common Python dev tools
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir \
        black \
        ruff \
        pip-tools

# Install Claude Code
RUN npm install -g @anthropic-ai/claude-code

# Set up workspace
WORKDIR /workspace

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Set environment variables for development
ENV PYTHONPATH=/workspace
ENV PYTHONUNBUFFERED=1