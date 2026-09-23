//! 3MF loader.
//!
//! A 3MF file is a zip archive holding one or more XML `.model` files. The root model lists
//! the build items to print, and each item places an object with a transform. Objects are
//! either a mesh or a list of components that place other objects, which may live in other
//! `.model` files (the production extension's `p:path` attribute, used by Bambu Studio and
//! other slicers).

use std::collections::HashMap;
use std::error::Error;
use std::io::{BufRead, BufReader, Read, Seek};

use cgmath::{Matrix4, Point3, SquareMatrix, Transform};
use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;
use zip::ZipArchive;

use crate::mesh::{Mesh, MeshBuilder};

/// Components deeper than this are treated as a cycle.
const MAX_DEPTH: usize = 32;

const DEFAULT_ROOT_MODEL: &str = "3d/3dmodel.model";

struct ModelFile {
    objects: HashMap<u64, Object>,
    build: Vec<Placement>,
}

#[derive(Default)]
struct Object {
    vertices: Vec<[f64; 3]>,
    triangles: Vec<[usize; 3]>,
    components: Vec<Placement>,
}

struct Placement {
    objectid: u64,
    /// Model file holding the object, when it is not the file holding the placement.
    path: Option<String>,
    transform: Matrix4<f64>,
}

pub fn load<R: Read + Seek>(reader: R) -> Result<Mesh, Box<dyn Error>> {
    let mut zip = ZipArchive::new(reader)?;

    let mut files = HashMap::new();
    for i in 0..zip.len() {
        let file = zip.by_index(i)?;
        if file.name().to_lowercase().ends_with(".model") {
            let path = normalize_path(file.name());
            let model = parse_model(BufReader::new(file))
                .map_err(|e| format!("Unable to read {} in 3MF file: {}", path, e))?;
            files.insert(path, model);
        }
    }

    let root_path = find_root_model(&mut zip, &files)?;
    let root = &files[&root_path];

    let mut builder = MeshBuilder::new();
    if root.build.is_empty() {
        // Not valid 3MF, but show something rather than nothing.
        warn!("3MF file has no build items. Rendering every object instead");
        for id in root.objects.keys() {
            add_object(
                &mut builder,
                &files,
                &root_path,
                *id,
                Matrix4::identity(),
                0,
            )?;
        }
    } else {
        for item in &root.build {
            let path = item.path.as_deref().unwrap_or(&root_path);
            add_object(&mut builder, &files, path, item.objectid, item.transform, 0)?;
        }
    }
    builder.finish("3MF")
}

/// Add an object and all of its components to the mesh, placed by `transform`.
fn add_object(
    builder: &mut MeshBuilder,
    files: &HashMap<String, ModelFile>,
    path: &str,
    id: u64,
    transform: Matrix4<f64>,
    depth: usize,
) -> Result<(), Box<dyn Error>> {
    if depth > MAX_DEPTH {
        return Err("3MF components are nested too deeply or form a cycle".into());
    }
    let Some(object) = files.get(path).and_then(|file| file.objects.get(&id)) else {
        warn!(
            "3MF file references object {} in {}, which does not exist",
            id, path
        );
        return Ok(());
    };

    let vertices: Vec<[f32; 3]> = object
        .vertices
        .iter()
        .map(|v| {
            let p = transform.transform_point(Point3::new(v[0], v[1], v[2]));
            [p.x as f32, p.y as f32, p.z as f32]
        })
        .collect();
    // A mirroring transform turns counter-clockwise triangles clockwise, so swap them back.
    let mirrored = transform.determinant() < 0.0;
    for triangle in &object.triangles {
        let vertex = |index: usize| {
            vertices
                .get(triangle[index])
                .copied()
                .ok_or("3MF triangle references a vertex that does not exist")
        };
        let (a, b, c) = if mirrored { (0, 2, 1) } else { (0, 1, 2) };
        builder.add_triangle([vertex(a)?, vertex(b)?, vertex(c)?], None);
    }

    for component in &object.components {
        let path = component.path.as_deref().unwrap_or(path);
        add_object(
            builder,
            files,
            path,
            component.objectid,
            transform * component.transform,
            depth + 1,
        )?;
    }
    Ok(())
}

/// Find the root model from the package relationships, falling back to the standard location.
fn find_root_model<R: Read + Seek>(
    zip: &mut ZipArchive<R>,
    files: &HashMap<String, ModelFile>,
) -> Result<String, Box<dyn Error>> {
    if let Ok(rels) = zip.by_name("_rels/.rels") {
        let mut reader = Reader::from_reader(BufReader::new(rels));
        let mut buf = Vec::new();
        loop {
            match reader.read_event_into(&mut buf)? {
                Event::Start(e) | Event::Empty(e) if e.local_name().as_ref() == b"Relationship" => {
                    let is_model = attribute(&e, b"Type")?.is_some_and(|t| t.ends_with("/3dmodel"));
                    if let Some(target) = attribute(&e, b"Target")?.filter(|_| is_model) {
                        let path = normalize_path(&target);
                        if files.contains_key(&path) {
                            return Ok(path);
                        }
                    }
                }
                Event::Eof => break,
                _ => (),
            }
            buf.clear();
        }
    }
    if files.contains_key(DEFAULT_ROOT_MODEL) {
        return Ok(DEFAULT_ROOT_MODEL.to_string());
    }
    match files.keys().next() {
        Some(path) if files.len() == 1 => Ok(path.clone()),
        _ => Err("Unable to find the root model in 3MF file".into()),
    }
}

fn parse_model<R: BufRead>(reader: R) -> Result<ModelFile, Box<dyn Error>> {
    let mut reader = Reader::from_reader(reader);
    let mut buf = Vec::new();
    let mut model = ModelFile {
        objects: HashMap::new(),
        build: Vec::new(),
    };
    // Object currently being read, and whether we are inside its <mesh> or the <build>.
    // Checking these keeps extension elements with the same names from being picked up.
    let mut object: Option<(u64, Object)> = None;
    let mut in_mesh = false;
    let mut in_build = false;

    loop {
        let event = reader.read_event_into(&mut buf)?;
        match &event {
            Event::Start(e) | Event::Empty(e) => {
                let empty = matches!(event, Event::Empty(_));
                match e.local_name().as_ref() {
                    b"object" => {
                        let [id] = numbers(e, [b"id"])?;
                        let current = (id, Object::default());
                        if empty {
                            model.objects.insert(current.0, current.1);
                        } else {
                            object = Some(current);
                        }
                    }
                    b"mesh" => in_mesh = !empty,
                    b"build" => in_build = !empty,
                    b"vertex" if in_mesh => {
                        if let Some((_, object)) = &mut object {
                            object.vertices.push(numbers(e, [b"x", b"y", b"z"])?);
                        }
                    }
                    b"triangle" if in_mesh => {
                        if let Some((_, object)) = &mut object {
                            object.triangles.push(numbers(e, [b"v1", b"v2", b"v3"])?);
                        }
                    }
                    b"component" => {
                        if let Some((_, object)) = &mut object {
                            object.components.push(placement(e)?);
                        }
                    }
                    b"item" if in_build => model.build.push(placement(e)?),
                    _ => (),
                }
            }
            Event::End(e) => match e.local_name().as_ref() {
                b"object" => {
                    if let Some((id, object)) = object.take() {
                        model.objects.insert(id, object);
                    }
                }
                b"mesh" => in_mesh = false,
                b"build" => in_build = false,
                _ => (),
            },
            Event::Eof => break,
            _ => (),
        }
        buf.clear();
    }
    Ok(model)
}

fn placement(e: &BytesStart) -> Result<Placement, Box<dyn Error>> {
    let transform = match attribute(e, b"transform")? {
        Some(transform) => parse_transform(&transform)?,
        None => Matrix4::identity(),
    };
    Ok(Placement {
        objectid: numbers(e, [b"objectid"])?[0],
        path: attribute(e, b"path")?.map(|path| normalize_path(&path)),
        transform,
    })
}

/// Parse a 3MF transform: a 4x3 matrix of 12 numbers, applied to row vectors.
fn parse_transform(s: &str) -> Result<Matrix4<f64>, Box<dyn Error>> {
    let m = s
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| format!("Invalid 3MF transform {:?}", s))?;
    if m.len() != 12 {
        return Err(format!("Invalid 3MF transform {:?}", s).into());
    }
    // cgmath matrices are column major and applied to column vectors, which lays the
    // numbers out in the same order as 3MF's row major matrices applied to row vectors.
    #[rustfmt::skip]
    let matrix = Matrix4::new(
        m[0], m[1], m[2], 0.0,
        m[3], m[4], m[5], 0.0,
        m[6], m[7], m[8], 0.0,
        m[9], m[10], m[11], 1.0,
    );
    Ok(matrix)
}

/// Look up an attribute by its name without any namespace prefix.
fn attribute(e: &BytesStart, name: &[u8]) -> Result<Option<String>, Box<dyn Error>> {
    for attr in e.attributes() {
        let attr = attr?;
        if attr.key.local_name().as_ref() == name {
            return Ok(Some(String::from_utf8(attr.value.into_owned())?));
        }
    }
    Ok(None)
}

/// Parse the named numeric attributes of an element in a single pass over its attributes.
fn numbers<T: std::str::FromStr + Copy + Default, const N: usize>(
    e: &BytesStart,
    names: [&[u8]; N],
) -> Result<[T; N], Box<dyn Error>> {
    let mut values = [None; N];
    for attr in e.attributes() {
        let attr = attr?;
        if let Some(i) = names
            .iter()
            .position(|n| *n == attr.key.local_name().as_ref())
        {
            let value = std::str::from_utf8(&attr.value).ok();
            values[i] = Some(value.and_then(|v| v.trim().parse().ok()).ok_or_else(|| {
                format!(
                    "3MF {} has an invalid {} {:?}",
                    String::from_utf8_lossy(e.local_name().as_ref()),
                    String::from_utf8_lossy(names[i]),
                    String::from_utf8_lossy(&attr.value)
                )
            })?);
        }
    }
    let mut result = [T::default(); N];
    for (i, value) in values.into_iter().enumerate() {
        result[i] = value.ok_or_else(|| {
            format!(
                "3MF {} is missing its {} attribute",
                String::from_utf8_lossy(e.local_name().as_ref()),
                String::from_utf8_lossy(names[i])
            )
        })?;
    }
    Ok(result)
}

/// Paths inside the package are case-insensitive and may start with a slash.
fn normalize_path(path: &str) -> String {
    path.trim_start_matches('/').to_lowercase()
}
