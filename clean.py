# clean_requirements.py

input_file = "requirements.txt"
output_file = "clean_requirements.txt"

with open(input_file, "r") as fin, open(output_file, "w") as fout:
    for line in fin:
        if "@ file://" not in line and not line.startswith("-e git+"):
            fout.write(line)
        elif line.startswith("-e git+"):  # keep editable Git installs if needed
            fout.write(line)

print(f"Cleaned file saved to {output_file}")
