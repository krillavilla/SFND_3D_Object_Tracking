#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def plot_ttc_comparison(csv_file, output_file):
    """
    Generate a comparison plot of TTC values for different detector/descriptor combinations.

    Args:
        csv_file: Path to the CSV file containing TTC results
        output_file: Path to save the output plot
    """
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    # Read the CSV file
    df = pd.read_csv(csv_file)

    # Create a unique identifier for each detector/descriptor combination
    df['Combination'] = df['DetectorType'] + '+' + df['DescriptorType']

    # Get unique combinations and frames
    combinations = df['Combination'].unique()
    frames = sorted(df['Frame'].unique())

    # Print available combinations
    print(f"Available detector/descriptor combinations: {combinations}")

    # Create a figure with two subplots (Lidar and Camera TTC)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Plot Lidar TTC
    for combo in combinations:
        combo_data = df[df['Combination'] == combo]
        # Group by frame and calculate mean TTC (in case there are multiple boxes per frame)
        frame_data = combo_data.groupby('Frame')['TTCLidar'].mean().reset_index()
        ax1.plot(frame_data['Frame'], frame_data['TTCLidar'], marker='o', label=combo)

    ax1.set_title('Lidar-based TTC Estimation')
    ax1.set_xlabel('Frame')
    ax1.set_ylabel('TTC (seconds)')
    ax1.grid(True)
    ax1.legend(loc='upper right')

    # Plot Camera TTC
    for combo in combinations:
        combo_data = df[df['Combination'] == combo]
        # Group by frame and calculate mean TTC (in case there are multiple boxes per frame)
        frame_data = combo_data.groupby('Frame')['TTCCamera'].mean().reset_index()
        ax2.plot(frame_data['Frame'], frame_data['TTCCamera'], marker='o', label=combo)

    ax2.set_title('Camera-based TTC Estimation')
    ax2.set_xlabel('Frame')
    ax2.set_ylabel('TTC (seconds)')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    print(f"Plot saved to {output_file}")

    # Generate a table of TTC values for the README
    table_df = pd.DataFrame(index=frames)

    for combo in combinations:
        combo_data = df[df['Combination'] == combo]
        frame_data = combo_data.groupby('Frame')['TTCCamera'].mean()
        table_df[combo] = frame_data

    # Save the table to a CSV file
    table_file = os.path.splitext(output_file)[0] + '_table.csv'
    table_df.to_csv(table_file)
    print(f"Table saved to {table_file}")

    # Print the first few rows of the table for the README
    print("\nCamera TTC Table (first few rows):")
    print(table_df.head().to_csv())

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_ttc.py <results_csv> <output_png>")
        sys.exit(1)

    csv_file = sys.argv[1]
    output_file = sys.argv[2]

    plot_ttc_comparison(csv_file, output_file)
