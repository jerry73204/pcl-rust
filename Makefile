# Simple build and test automation

.PHONY: build test build-default test-default build-all-features test-all-features clean help lint

# Default target
all: build

# Aliases for convenience
build: build-default
test: test-default

# Build the entire Rust workspace with all targets
build-default:
	cargo build --all-targets

# Run tests without CARLA server connection
test-default:
	cargo nextest run --all-targets --no-fail-fast

# Clean build artifacts
clean:
	cargo clean

# Build with all features
build-all-features:
	cargo build --all-targets --all-features

# Test with all features (visualization is disabled)
test-all-features:
	cargo nextest run --all-targets --all-features --no-fail-fast

lint:
	cargo clippy --all-targets --all-features

# Show available targets
help:
	@echo "Available targets:"
	@echo "  build-default     - Build the Rust project with --all-targets (default features)"
	@echo "  test-default      - Run tests with default features"
	@echo "  build-all-features - Build with all features (visualization is disabled)"
	@echo "  test-all-features - Run tests with all features (visualization is disabled)"
	@echo "  lint              - Run clippy for code quality checks"
	@echo "  clean             - Clean up build artifacts"
	@echo "  help              - Show this help message"
