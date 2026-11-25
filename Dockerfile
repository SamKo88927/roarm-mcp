# Runs the simulated arm over stdio. Used by MCP directories (e.g. Glama) to start and inspect the server.
FROM python:3.13-slim
ENV PIP_NO_CACHE_DIR=1 PYTHONUNBUFFERED=1
WORKDIR /app
COPY pyproject.toml README.md LICENSE ./
COPY src ./src
RUN pip install --no-compile .
RUN useradd --create-home roarm
USER roarm
ENTRYPOINT ["roarm-mcp", "--fast"]
