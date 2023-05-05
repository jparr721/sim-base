import typer
import numpy as np
import os
from rich import print
from rich.progress import track
import copy

app = typer.Typer()

scene_drop = """
<scene version="3.0.0">

    <!-- Defaults, these can be set via the command line: -Darg=value -->

    <default name="spp" value="300"/>
    <default name="resx" value="1000"/>
    <default name="resy" value="800"/>
    <default name="max_depth" value="8"/>


    <!--    Radius 17.5-->
    <!--    Theta -0.0391999-->
    <!--    Phi 1.7608-->
    <!--    Eye 12.0753, -3.86577, 8.59707-->
    <!--    Look At -5.09655, -0.560674, 9.27055-->
    <!--    Up 0.188717, 0.982004, -0.0074015-->
    <!--    FOV 65-->

    <!-- Maybe another option -->
    <!--    Eye 9.21331, -1.59174, 8.86574-->
    <!--    Look At 0.38209, 0.10802, 9.2121-->
    <!--    Up 0.188717, 0.982004, -0.00740149-->
    <!--    FOV 65-->

    <!-- This one is good -->
    <!--    Eye 12.1386, -1.41318, 9.85186-->
    <!--    Look At 0.463037, 0.233464, 10.3098-->
    <!--    Up 0.13944, 0.990216, -0.00546882-->
    <!--    FOV 65-->

    <!-- Fast Drop -->
    <!--    ->SetRadius(11.7)-->
    <!--    ->SetTheta(-0.0091999)-->
    <!--    ->SetPhi(1.7208)-->
    <!--    ->SetDisplacement(Vec3<Real>(-1.04227e+17, 6.4785e+22, 3.32053e+27))-->
    <!--    Eye 11.9113, -2.16091, 10.6684-->
    <!--    Look At 0.343205, -0.412442, 10.7748-->
    <!--    Up 0.149435, 0.988771, -0.00137483-->
    <!--    FOV 65-->
    
    <!-- Camera and Rendering Parameters -->

    <integrator type="path">
        <boolean name="hide_emitters" value="true"/>
        <integer name="max_depth" value="$max_depth"/>
    </integrator>
    <sensor type="perspective">
        <string name="fov_axis" value="y"/>
        <float name="fov" value="65"/>
        <transform name="to_world">
            <lookat origin="6.0885, -2.58997, 10.1868" target="-2.40966, -2.58994, 10.01"
                    up="3.73839e-06, 1, 7.77733e-08"/>
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
        <float name="scale" value="6"/>
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


def make_obj(filename: str):
    return f"""
    <shape type="obj">
        <string name="filename" value="{filename}"/>
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
        s = copy.deepcopy(scene_drop)

        if 'txt' in file:
            comb_filename = file.replace("frame", "comb")
            comb_filename = comb_filename.replace(".txt", ".obj")
            s += make_bspline(file)
            s += make_obj(comb_filename)

        s += "</scene>\n"
        fn = os.path.splitext(file)[0]
        with open(os.path.join(folder, fn + ".xml"), "w") as f:
            f.write(s)


if __name__ == "__main__":
    app()
