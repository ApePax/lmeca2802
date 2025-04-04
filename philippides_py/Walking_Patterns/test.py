import pandas as pd

def keep_first_five_columns_overwrite(csv_file_path):
    """
    Reads a CSV file, keeps only the first five columns, and overwrites the original file.

    Args:
        csv_file_path (str): The path to the CSV file.
    """
    try:
        df = pd.read_csv(csv_file_path)
        first_five_columns = df.iloc[:, :5]
        first_five_columns.to_csv(csv_file_path, index=False)  # Overwrite the original file

        print(f"Successfully extracted the first 5 columns and overwrote {csv_file_path}")

    except FileNotFoundError:
        print(f"Error: File not found at {csv_file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage:
csv_file = "WP.csv"  # Replace with your CSV file path
keep_first_five_columns_overwrite(csv_file)