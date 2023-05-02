import os
from dataclasses import dataclass
from typing import List

import typer
import numpy as np

app = typer.Typer()


def normalize(v: np.ndarray):
    return v / np.linalg.norm(v)


class PbrtCylinder:
    def __init__(self,
                 bottom: np.ndarray,
                 top: np.ndarray,
                 radius: float,
                 *,
                 rotation_angle: np.array = None,
                 rotation_axis: np.ndarray = None):
        self.bottom = bottom
        self.top = top
        self.radius = radius
        self.rotation_angle = rotation_angle
        self.rotation_axis = rotation_axis

        if self.rotation_axis is None or self.rotation_angle is None:
            self._get_cylinder_rotation(bottom, top)

    def to_pbrt(self):
        s = "AttributeBegin\n"
        s += 'Material "hair"\n'
        s += f"Translate {self.bottom[0]} {self.bottom[1]} {self.bottom[2]}\n"
        s += f"Rotate {self.rotation_angle} {self.rotation_axis[0]} {self.rotation_axis[1]} {self.rotation_axis[2]}\n"
        s += f'Shape "cylinder" "float radius" [{self.radius}] "float zmin" [0]' + \
             f'"float zmax" [{np.linalg.norm(self.top - self.bottom)}]\n'
        s += "AttributeEnd\n"
        return s

    def _get_cylinder_rotation(self, v1: np.ndarray, v2: np.ndarray):
        direction = v2 - v1
        length = np.linalg.norm(direction)
        z = np.array([0, 0, 1])
        axis = np.cross(z, direction)
        angle = np.arccos(np.dot(normalize(direction), z) / np.linalg.norm(v1) * np.linalg.norm(v2))
        if np.isnan(angle):
            angle = 0
        self.rotation_angle = angle * 180 / np.pi
        self.rotation_axis = axis


def from_obj(filename: str) -> List[PbrtCylinder]:
    cylinders = []
    with open(filename) as f:
        vertices = []
        for line in f.readlines():
            if line.lower().startswith("v "):
                v = np.array([float(x) for x in line.split()[1:]])
                vertices.append(v)

        if len(vertices) % 2 != 0:
            raise ValueError("Number of vertices must be even")

        for ii in range(0, len(vertices), 2):
            cylinders.append(PbrtCylinder(vertices[ii], vertices[ii + 1], 0.01))

    return cylinders


def from_folder(dirname: str) -> List[PbrtCylinder]:
    if not os.path.exists(dirname):
        raise ValueError(f"Directory {dirname} does not exist")

    cylinders = []
    for file in os.listdir(dirname):
        cylinders.extend(from_obj(os.path.join(dirname, file)))
    return cylinders


@dataclass
class PbrtScene:
    frameno: int
    eye: np.ndarray
    look_at: np.ndarray
    up: np.ndarray
    fov: float
    samples: int
    max_depth: int
    width: int
    height: int
    output_filename: str
    cylinders: List[PbrtCylinder]

    def to_pbrt(self):
        pbrt_filename = f"frame_{self.frameno}.pbrt"
        with open(pbrt_filename, "w+") as scene:
            eye = f"{self.eye[0]} {self.eye[1]} {self.eye[2]}\n"
            look_at = f"{self.look_at[0]} {self.look_at[1]} {self.look_at[2]}\n"
            up = f"{self.up[0]} {self.up[1]} {self.up[2]}\n"
            campos = f"LookAt {eye} {look_at} {up}\n"
            camera = f'Camera "perspective" "float fov" [{self.fov}]\n'
            sampler = f'Sampler "halton" "integer pixelsamples" {self.samples}\n'
            integrator = f'Integrator "path" "integer maxdepth" {self.max_depth}\n'
            film = f'Film "rgb" "string filename" "{self.output_filename}"' + \
                   f'"integer xresolution" [{self.width}] "integer yresolution" [{self.height}]\n'

            world_begin = "WorldBegin\n"
            light_source = 'LightSource "point" "blackbody I" [10000]\n'
            cyl = "".join([c.to_pbrt() for c in self.cylinders])

            scene.write(campos)
            scene.write(camera)
            scene.write(sampler)
            scene.write(integrator)
            scene.write(film)
            scene.write(world_begin)
            scene.write(light_source)
            scene.write(cyl)


@app.command()
def render(scene_dir: str, output_dir: str):
    eye = np.array([26.2992, -2.97439, 41.9482])
    look_at = np.array([0, 0, 0])
    up = np.array([0.0318538, 0.9982, 0.050808])
    fov = 65
    cylinders = from_folder(scene_dir)
    scene = PbrtScene(0, eye, look_at, up, fov, 256, 12, 1920, 1080, output_dir, cylinders)
    scene.to_pbrt()


if __name__ == "__main__":
    content = """
LookAt 3 4 1.5  # eye
   .5 .5 0  # look at point
   0 0 1    # up vector
Camera "perspective" "float fov" 45

Sampler "halton" "integer pixelsamples" 128
Integrator "path"
Film "image" "string filename" "simple.png"
     "integer xresolution" [400] "integer yresolution" [400]

WorldBegin

# uniform blue-ish illumination from all directions
LightSource "infinite" "rgb L" [.4 .45 .5]

# approximate the sun
LightSource "distant"  "point from" [ -30 40  100 ]
   "blackbody L" [3000 1.5]

AttributeBegin
  Material "glass"
  Shape "sphere" "float radius" 1
AttributeEnd

AttributeBegin
  Texture "checks" "spectrum" "checkerboard"
          "float uscale" [8] "float vscale" [8]
          "rgb tex1" [.1 .1 .1] "rgb tex2" [.8 .8 .8]
  Material "matte" "texture Kd" "checks"
  Translate 0 0 -1
  Shape "trianglemesh"
      "integer indices" [0 1 2 0 2 3]
      "point P" [ -20 -20 0   20 -20 0   20 20 0   -20 20 0 ]
      "float st" [ 0 0   1 0    1 1   0 1 ]
AttributeEnd
    """

    for ii in range(300000):
        x = np.random.uniform(0, 1)
        y = np.random.uniform(0, 1)
        z = np.random.uniform(0, 1)

        sphere = f"""
    AttributeBegin
        Material "glass"
        Translate {x} {y} {z}
        Shape "sphere" "float radius" 1
    AttributeEnd\n\n
        """
        content += sphere

    content += "\nWorldEnd"

    with open(os.path.join(os.path.expanduser("~"), "Downloads", "frame.pbrt"), "w+") as f:
        f.write(content)
