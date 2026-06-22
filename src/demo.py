import argparse


def main():
    parser = argparse.ArgumentParser(description="Run a demo for the vision-guided robot sorting system.")
    parser.add_argument("--config", default="config/demo.yaml", help="Path to the configuration file.")
    args = parser.parse_args()

    print(f"Demo runner initialized with config: {args.config}")
    print("This repository contains the main source scaffold for vision-guided robot sorting.")
    print("Replace this file with your actual experiment or demo entrypoint.")


if __name__ == "__main__":
    main()
