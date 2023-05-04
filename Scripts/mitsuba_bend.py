import typer
import numpy as np
import os
from rich import print
from rich.progress import track

app = typer.Typer()
# Eye 1.88, -0.0300002, 4.4
# Look At 1.88, -0.03, 8.42896e-08
# Up -1.91069e-15, 1, 4.37114e-08
# FOV 65

scene_bend = """
<scene version="3.0.0">
    <!-- Defaults, these can be set via the command line: -Darg=value -->

    <default name="spp" value="300"/>
    <default name="resx" value="1000"/>
    <default name="resy" value="800"/>
    <default name="max_depth" value="8"/>

    <!-- Camera and Rendering Parameters -->

    <integrator type="path">
        <boolean name="hide_emitters" value="true"/>
        <integer name="max_depth" value="$max_depth"/>
    </integrator>
    <sensor type="perspective">
        <string name="fov_axis" value="y"/>
        <float name="fov" value="65"/>
        <transform name="to_world">
            <lookat origin="1.88, -0.0300002, 4.4" target="1.88, -0.03, 0"
                    up="0, 1, 0"/>
        </transform>
        <sampler type="multijitter">
            <integer name="sample_count" value="$spp"/>
        </sampler>
        <film type="hdrfilm">
            <string name="pixel_format" value="rgba"/>
            <rfilter type="gaussian"/>
            <integer name="width" value="$resx"/>
            <integer name="height" value="$resy"/>
        </film>
    </sensor>

    <!-- Materials -->
    <bsdf type="twosided">
        <bsdf type="roughplastic" id="brown">
            <float name="alpha" value="0.01"/>
            <string name="distribution" value="ggx"/>
            <float name="int_ior" value="1.55"/>
            <float name="ext_ior" value="1"/>
            <boolean name="nonlinear" value="false"/>
            <!-- BROWN HAIR -->
            <rgb name="diffuse_reflectance" value="0.143016, 0.0156076, 1.80928e-005"/>
        </bsdf>
    </bsdf>

    <!-- Emitters -->
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="../envmap.exr"/>
        <float name="scale" value="5"/>
    </emitter>
"""


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
    with open(output_filename, "w") as f:
        f.write(spline)


def make_bspline(filename: str):
    return f"""
    <shape type="linearcurve">
        <string name="filename" value="{filename}"/>
    </shape>
    """


def make_sphere(radius: float, cx, cy, cz):
    return f"""
    <shape type="sphere">
        <float name="radius" value="{radius}"/>
        <point name="center" x="{cx}" y="{cy}" z="{cz}"/>
    </shape>
    """


@app.command()
def make_render_scene(folder: str, radius: float):
    if not os.path.exists(folder):
        raise typer.BadParameter(f"Folder {folder} does not exist!")

    for file in track(os.listdir(folder)):
        # Convert frames to splines in the designated folder
        if file.startswith("frame") and file.endswith(".obj"):
            load_obj_into_spline(os.path.join(folder, file), radius)

    # Now create the spline entry and dump the scenes
    for file in track(os.listdir(folder)):
        if "txt" not in file:
            continue
        # Now create the scene, first by copying the template
        s = scene_bend

        if "txt" in file:
            # Holds the spline geometry
            txt_file = os.path.join(folder, file)
            s += make_bspline(txt_file)

            # Also add a sphere to the scene using the first and last values
            # of the spline geometry file
            with open(txt_file, "r") as f:
                center1 = f.readline().split()[0:3]
                # The last line is always empty, so take the previous
                center2 = f.readlines()[-2].split()[0:3]
                s += make_sphere(radius, *center1)
                s += make_sphere(radius, *center2)

        s += "</scene>\n"

        # Filename without the extension
        fn = os.path.splitext(file)[0]
        with open(os.path.join(folder, fn + ".xml"), "w") as f:
            f.write(s)


if __name__ == "__main__":
    app()
