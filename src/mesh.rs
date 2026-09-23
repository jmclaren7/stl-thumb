extern crate ahash;
extern crate cgmath;
extern crate stl_io;
extern crate tobj;

use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::io::{Cursor, Read, Seek};
use std::{fmt, io};

use self::stl_io::{Triangle, Vector};

use self::ahash::AHashMap;
use self::tobj::LoadOptions;

#[cfg(feature = "step")]
use crate::step;
use crate::three_mf;

#[derive(Copy, Clone)]
pub struct Vertex {
    position: [f32; 3],
    //texcoords: [f32; 2],
}

implement_vertex!(Vertex, position);
//implement_vertex!(Vertex, position, texcoords);

#[derive(Debug, Copy, Clone)]
pub struct Normal {
    normal: [f32; 3],
}

implement_vertex!(Normal, normal);

#[derive(Clone)]
pub struct BoundingBox {
    pub min: cgmath::Point3<f32>,
    pub max: cgmath::Point3<f32>,
}

impl BoundingBox {
    fn new(vert: &stl_io::Vertex) -> BoundingBox {
        BoundingBox {
            min: cgmath::Point3 {
                x: vert[0],
                y: vert[1],
                z: vert[2],
            },
            max: cgmath::Point3 {
                x: vert[0],
                y: vert[1],
                z: vert[2],
            },
        }
    }
    fn expand(&mut self, vert: &stl_io::Vertex) {
        if vert[0] < self.min.x {
            self.min.x = vert[0];
        } else if vert[0] > self.max.x {
            self.max.x = vert[0];
        }
        if vert[1] < self.min.y {
            self.min.y = vert[1];
        } else if vert[1] > self.max.y {
            self.max.y = vert[1];
        }
        if vert[2] < self.min.z {
            self.min.z = vert[2];
        } else if vert[2] > self.max.z {
            self.max.z = vert[2];
        }
    }
    pub fn center(&self) -> cgmath::Point3<f32> {
        cgmath::Point3 {
            x: (self.min.x + self.max.x) / 2.0,
            y: (self.min.y + self.max.y) / 2.0,
            z: (self.min.z + self.max.z) / 2.0,
        }
    }
    fn length(&self) -> f32 {
        self.max.x - self.min.x
    }
    fn width(&self) -> f32 {
        self.max.y - self.min.y
    }
    fn height(&self) -> f32 {
        self.max.z - self.min.z
    }
}

impl fmt::Display for BoundingBox {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "X: {}, {}", self.min.x, self.max.x)?;
        writeln!(f, "Y: {}, {}", self.min.y, self.max.y)?;
        writeln!(f, "Z: {}, {}", self.min.z, self.max.z)?;
        Ok(())
    }
}

#[derive(Clone)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub normals: Vec<Normal>,
    pub indices: Vec<usize>,
    pub bounds: BoundingBox,
    model_had_normals: bool,
}

impl Mesh {
    // Load mesh data from file (if provided) or stdin
    pub fn load(model_filename: &str, recalc_normals: bool) -> Result<Mesh, Box<dyn Error>> {
        // TODO: Add support for URIs instead of plain file names
        // https://developer.gnome.org/integration-guide/stable/thumbnailer.html.en
        match model_filename {
            "-" => {
                // create_stl_reader requires Seek, so we must read the entire stream into memory before proceeding.
                // So I guess this can just consume all RAM if it gets bad input. Hmmm....
                let mut input_buffer = Vec::new();
                io::stdin().read_to_end(&mut input_buffer)?;
                Mesh::from_stl(Cursor::new(input_buffer), recalc_normals)
            }
            _ => {
                let model_filename = std::path::Path::new(model_filename);
                let extension = model_filename
                    .extension()
                    .and_then(std::ffi::OsStr::to_str)
                    .unwrap_or("")
                    .to_lowercase();
                if !matches!(extension.as_str(), "obj" | "stl" | "3mf" | "step" | "stp") {
                    return Err(format!(
                        "Unsupported model format {:?}. Supported formats are STL, OBJ, 3MF and STEP",
                        extension
                    )
                    .into());
                }
                // No BufReader needed here: stl_io buffers internally, from_obj wraps the file itself
                // and the 3MF and STEP loaders buffer or read the whole file themselves
                let model_file = File::open(model_filename)?;
                match extension.as_str() {
                    "obj" => Mesh::from_obj(model_file, recalc_normals),
                    "stl" => Mesh::from_stl(model_file, recalc_normals),
                    "3mf" => Mesh::from_3mf(model_file, recalc_normals),
                    _ => Mesh::from_step(model_file, recalc_normals),
                }
            }
        }
    }

    pub fn from_3mf<R>(model_file: R, _recalc_normals: bool) -> Result<Mesh, Box<dyn Error>>
    where
        R: Read + Seek,
    {
        three_mf::load(model_file)
    }

    #[cfg(feature = "step")]
    pub fn from_step<R>(model_file: R, _recalc_normals: bool) -> Result<Mesh, Box<dyn Error>>
    where
        R: Read,
    {
        step::load(model_file)
    }

    #[cfg(not(feature = "step"))]
    pub fn from_step<R>(_model_file: R, _recalc_normals: bool) -> Result<Mesh, Box<dyn Error>>
    where
        R: Read,
    {
        Err("STEP support was not enabled when stl-thumb was built".into())
    }

    pub fn from_stl<R>(mut model_file: R, recalc_normals: bool) -> Result<Mesh, Box<dyn Error>>
    where
        R: Read + Seek,
    {
        //let model = stl_io::read_stl(&mut model_file)?;
        //debug!("{:?}", model);
        let mut stl_iter = stl_io::create_stl_reader(&mut model_file)?;

        // Get starting point for finding bounding box
        let t1 = stl_iter.next().ok_or("STL file contains no triangles")??;
        let v1 = t1.vertices[0];

        let mut mesh = Mesh {
            vertices: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
            bounds: BoundingBox::new(&v1),
            model_had_normals: true,
        };

        let mut face_count = 0;
        mesh.process_tri(&t1, recalc_normals);
        face_count += 1;

        for triangle in stl_iter {
            mesh.process_tri(&triangle?, recalc_normals);
            face_count += 1;
            //debug!("{:?}",triangle);
        }

        if !mesh.model_had_normals {
            warn!("STL file missing surface normals");
        }
        info!("Bounds:");
        info!("{}", mesh.bounds);
        info!("Center:\t{:?}", mesh.bounds.center());
        info!("Triangles processed:\t{}\n", face_count);

        Ok(mesh)
    }

    pub fn from_obj(obj_file: File, _recalc_normals: bool) -> Result<Mesh, Box<dyn Error>> {
        let mut model = BufReader::new(obj_file);
        let (models, _) = tobj::load_obj_buf(
            &mut model,
            &LoadOptions {
                single_index: true,
                triangulate: true,
                ..LoadOptions::default()
            },
            |_| Ok((Vec::new(), AHashMap::new())),
        )?;
        let first_mesh = &models.first().ok_or("Empty Model")?.mesh;
        let mut first_vertex = first_mesh.positions.iter();
        let mut mesh = Mesh {
            vertices: Vec::with_capacity(first_mesh.positions.len() / 3),
            normals: Vec::with_capacity(first_mesh.normals.len() / 3),
            indices: Vec::with_capacity(first_mesh.indices.len() / 3),
            bounds: BoundingBox::new(&Vector::new([
                *first_vertex.next().ok_or("Empty Mesh")?,
                *first_vertex.next().ok_or("Empty Mesh")?,
                *first_vertex.next().ok_or("Empty Mesh")?,
            ])),
            model_had_normals: true,
        };
        for model in &models {
            let tri_idx = &model.mesh.indices;
            let p = &model.mesh.positions;
            let n = &model.mesh.normals;
            for i in (0..tri_idx.len()).step_by(3) {
                let index0: usize = tri_idx[i] as usize;
                let index1: usize = tri_idx[i + 1] as usize;
                let index2: usize = tri_idx[i + 2] as usize;

                let vertices = [
                    Vector::new([p[index0 * 3], p[index0 * 3 + 1], p[index0 * 3 + 2]]),
                    Vector::new([p[index1 * 3], p[index1 * 3 + 1], p[index1 * 3 + 2]]),
                    Vector::new([p[index2 * 3], p[index2 * 3 + 1], p[index2 * 3 + 2]]),
                ];
                for v in vertices.iter() {
                    mesh.bounds.expand(v);
                    mesh.vertices.push(Vertex {
                        position: (*v).into(),
                    });
                    //debug!("{:?}", v);
                }

                let normals = if !n.is_empty() {
                    [
                        Normal {
                            normal: ([n[index0 * 3], n[index0 * 3 + 1], n[index0 * 3 + 2]]),
                        },
                        Normal {
                            normal: ([n[index1 * 3], n[index1 * 3 + 1], n[index1 * 3 + 2]]),
                        },
                        Normal {
                            normal: ([n[index2 * 3], n[index2 * 3 + 1], n[index2 * 3 + 2]]),
                        },
                    ]
                } else {
                    let n = normal(&Triangle {
                        vertices,
                        normal: Vector::new([0.0, 0.0, 0.0]),
                    });
                    [n, n, n]
                };
                for normal in normals.iter() {
                    mesh.normals.push(*normal);
                }
            }
        }
        Ok(mesh)
    }

    fn process_tri(&mut self, tri: &stl_io::Triangle, recalc_normals: bool) {
        for v in tri.vertices {
            self.bounds.expand(&v);
            self.vertices.push(Vertex { position: v.into() });
            //debug!("{:?}", v);
        }
        // Use normal from STL file if it is provided, otherwise calculate it ourselves
        let n: Normal;
        if recalc_normals || (tri.normal == stl_io::Vector::new([0.0, 0.0, 0.0])) {
            self.model_had_normals = false;
            n = normal(tri);
        } else {
            n = Normal {
                normal: tri.normal.into(),
            };
        }
        //debug!("{:?}",tri.normal);
        // TODO: Figure out how to get away with 1 normal instead of 3
        for _ in 0..3 {
            self.normals.push(n);
        }
    }

    // Move the mesh to be centered at the origin
    // and scaled to fit a 2 x 2 x 2 box. This means that
    // all coordinates will be between -1.0 and 1.0
    pub fn scale_and_center(&self) -> cgmath::Matrix4<f32> {
        // Move center to origin
        let center = self.bounds.center();
        let translation_vector = cgmath::Vector3::new(-center.x, -center.y, -center.z);
        let translation_matrix = cgmath::Matrix4::from_translation(translation_vector);
        // Scale
        let longest = self
            .bounds
            .length()
            .max(self.bounds.width())
            .max(self.bounds.height());
        let scale = 2.0 / longest;
        info!("Scale:\t{}", scale);
        let scale_matrix = cgmath::Matrix4::from_scale(scale);
        scale_matrix * translation_matrix
    }
}

/// Collects triangles from the format loaders that build a mesh one triangle at a time.
pub(crate) struct MeshBuilder {
    mesh: Option<Mesh>,
    triangles: usize,
}

impl MeshBuilder {
    pub(crate) fn new() -> MeshBuilder {
        MeshBuilder {
            mesh: None,
            triangles: 0,
        }
    }

    /// Add a triangle whose vertices are in counter-clockwise order when seen from outside.
    /// Normals are per vertex. They are calculated from the triangle when not given.
    pub(crate) fn add_triangle(&mut self, vertices: [[f32; 3]; 3], normals: Option<[[f32; 3]; 3]>) {
        let vertices = vertices.map(stl_io::Vertex::new);
        let mesh = self.mesh.get_or_insert_with(|| Mesh {
            vertices: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
            bounds: BoundingBox::new(&vertices[0]),
            model_had_normals: normals.is_some(),
        });
        for v in &vertices {
            mesh.bounds.expand(v);
            mesh.vertices.push(Vertex {
                position: (*v).into(),
            });
        }
        match normals {
            Some(normals) => mesh.normals.extend(normals.map(|normal| Normal { normal })),
            None => {
                let n = normal(&Triangle {
                    normal: Vector::new([0.0, 0.0, 0.0]),
                    vertices,
                });
                mesh.normals.extend([n, n, n]);
            }
        }
        self.triangles += 1;
    }

    /// Finish the mesh. `format` names the file format in log and error messages.
    pub(crate) fn finish(self, format: &str) -> Result<Mesh, Box<dyn Error>> {
        let mesh = self
            .mesh
            .ok_or_else(|| format!("{} file contains no triangles", format))?;
        info!("Bounds:");
        info!("{}", mesh.bounds);
        info!("Center:\t{:?}", mesh.bounds.center());
        info!("Triangles processed:\t{}\n", self.triangles);
        Ok(mesh)
    }
}

impl fmt::Display for Mesh {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Verts: {}", self.vertices.len())?;
        writeln!(f, "Norms: {}", self.normals.len())?;
        //writeln!(f, "Tex Coords: {:?}", geometry.tex_coords)?;
        writeln!(f, "Indices: {:?}", self.indices.len())?;
        writeln!(f)?;
        Ok(())
    }
}

// Calculate surface normal of triangle using cross product
// TODO: The GPU can probably do this a lot faster than we can.
// See if there is an option for offloading this.
// Probably need to use a geometry shader (not supported in Opengl ES).
fn normal(tri: &stl_io::Triangle) -> Normal {
    let p1 = cgmath::Vector3::new(tri.vertices[0][0], tri.vertices[0][1], tri.vertices[0][2]);
    let p2 = cgmath::Vector3::new(tri.vertices[1][0], tri.vertices[1][1], tri.vertices[1][2]);
    let p3 = cgmath::Vector3::new(tri.vertices[2][0], tri.vertices[2][1], tri.vertices[2][2]);
    let v = p2 - p1;
    let w = p3 - p1;
    let n = v.cross(w);
    let mag = n.x.abs() + n.y.abs() + n.z.abs();
    Normal {
        normal: [n.x / mag, n.y / mag, n.z / mag],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cgmath::InnerSpace;

    fn triangles(mesh: &Mesh) -> usize {
        mesh.vertices.len() / 3
    }

    /// Enclosed volume, which is only positive when triangles wind counter-clockwise from outside.
    fn signed_volume(mesh: &Mesh) -> f32 {
        mesh.vertices
            .chunks(3)
            .map(|t| {
                let [a, b, c] = [t[0], t[1], t[2]].map(|v| cgmath::Vector3::from(v.position));
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }

    fn assert_bounds(mesh: &Mesh, min: [f32; 3], max: [f32; 3]) {
        let b = &mesh.bounds;
        for (actual, expected) in [b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]
            .iter()
            .zip(min.iter().chain(max.iter()))
        {
            assert!(
                (actual - expected).abs() < 0.01,
                "bounds {} do not match {:?} {:?}",
                b,
                min,
                max
            );
        }
    }

    #[test]
    fn threemf_components_and_transforms() {
        let mesh = Mesh::load("test_data/components.3mf", false).unwrap();
        assert_eq!(triangles(&mesh), 6 * 12);
        assert_bounds(&mesh, [-3.0, 0.0, 0.0], [4.0, 6.0, 1.0]);
        // A mirrored cube with the wrong winding would subtract its volume instead
        assert!((signed_volume(&mesh) - 6.0).abs() < 0.001);
    }

    #[test]
    fn threemf_bambu_multi_file() {
        let mesh = Mesh::load("test_data/nut_lever_bambu.3mf", false).unwrap();
        assert_eq!(triangles(&mesh), 1672);
        assert!(signed_volume(&mesh) > 0.0);
    }

    #[cfg(feature = "step")]
    #[test]
    fn step_assembly() {
        let mesh = Mesh::load("test_data/mount_assem1.step", false).unwrap();
        assert!(triangles(&mesh) > 1000);
        // The bounds of the assembled parts, which only match with the assembly transforms applied
        assert_bounds(&mesh, [-50.0, -7.0, -46.711], [50.0, 25.174, 61.072]);
        assert!(signed_volume(&mesh) > 0.0);
    }

    #[cfg(feature = "step")]
    #[test]
    fn step_extension_stp() {
        let dir = std::env::temp_dir().join("stl-thumb-test-stp");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("mount.STP");
        std::fs::copy("test_data/mount_assem1.step", &path).unwrap();
        assert!(Mesh::load(path.to_str().unwrap(), false).is_ok());
    }
}
