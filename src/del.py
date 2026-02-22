import math
import tkinter as tk
from tkinter import filedialog, messagebox


def read_file_with_auto_encoding(filepath):
    encodings = ['utf-8', 'utf-16', 'latin1', 'cp1252']
    for enc in encodings:
        try:
            with open(filepath, 'r', encoding=enc) as f:
                return f.readlines()
        except UnicodeDecodeError:
            continue
    raise ValueError("Could not decode file.")


def convert_grid_to_xyz(input_file, output_file):
    lines = read_file_with_auto_encoding(input_file)

    # ---- Metadata ----
    total_points = int(lines[4].strip())
    dx = float(lines[5].strip())
    dy = float(lines[6].strip())

    # ---- Read Z values (comma separated rows) ----
    z_values = []
    for line in lines[7:]:
        parts = line.strip().split(',')
        for val in parts:
            if val.strip() != '':
                z_values.append(float(val.strip()))

    if len(z_values) != total_points:
        raise ValueError(
            f"Declared {total_points} points but found {len(z_values)} values"
        )

    # ---- Compute Grid Size ----
    grid_size = int(math.sqrt(total_points))
    if grid_size * grid_size != total_points:
        raise ValueError("Total points is not a perfect square")

    # ---- Write XYZ ----
    with open(output_file, 'w') as out:
        for idx, z in enumerate(z_values):
            row = idx // grid_size
            col = idx % grid_size

            x = col * dx
            y = row * dy

            out.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def main():
    root = tk.Tk()
    root.withdraw()

    input_file = filedialog.askopenfilename(
        title="Select Grid TPP File",
        filetypes=[("TPP Files", "*.tpp"), ("All Files", "*.*")]
    )

    if not input_file:
        return

    output_file = filedialog.asksaveasfilename(
        title="Save XYZ File As",
        defaultextension=".xyz",
        filetypes=[("XYZ Files", "*.xyz")]
    )

    if not output_file:
        return

    try:
        convert_grid_to_xyz(input_file, output_file)
        messagebox.showinfo("Success", "XYZ file created successfully!")
    except Exception as e:
        messagebox.showerror("Error", str(e))


if __name__ == "__main__":
    main()
