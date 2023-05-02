import typer
import numpy as np
import os
from rich import print

app = typer.Typer()


def read_line_obj(obj_file: str):
    v = []
    l = []
    with open(obj_file, "r") as f:
        for line in f:
            if line.startswith("v"):
                v.append(np.array(line.split()[1:], dtype=np.float32))
            if line.startswith("l"):
                value = np.array(line.split()[1:], dtype=np.int32)
                value -= 1
                l.append(value)
    return np.array(v), np.array(l)


@app.command()
def load_obj_into_spline(obj_file: str, radius: float):
    if not os.path.exists(obj_file):
        raise typer.BadParameter(f"File {obj_file} does not exist!")

    v, l = read_line_obj(obj_file)
    if len(v) == 0:
        raise typer.BadParameter(f"File {obj_file} does not contain any vertices!")
    if len(l) == 0:
        raise typer.BadParameter(f"File {obj_file} does not contain any lines!")
    print(f"[green]Loaded {len(v)} vertices and {len(l)} lines from {obj_file}")
    print(f"[magenta]Shapes of vertices and lines: {v.shape}, {l.shape}")

    spline = ""
    for i, line in enumerate(l):
        for entry in line:
            vertex = v[entry]
            spline += f"{vertex[0]} {vertex[1]} {vertex[2]} {radius}\n"
        spline += "\n"

    output_filename = os.path.splitext(obj_file)[0] + ".txt"
    print("[magenta]Writing spline to", output_filename)
    with open("spline.txt", "w") as f:
        f.write(spline)


if __name__ == "__main__":
    app()
